/*
 * Copyright (c) 2007-2018 Immo Software
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * @file
 * @brief Implementation of Ar microkernel queue.
 */

#include "argon/queue.hpp"
#include "argon/thread.hpp"
#include "argon/kernel.hpp"
#include "argon/port.hpp"
#include "argon/assert.hpp"
#include <cstring>
#include <cstdint>

namespace Ar {

	//------------------------------------------------------------------------------
	// Macros
	//------------------------------------------------------------------------------

	//! Returns a uint8_t * to the first byte of element @a i of the queue.
	//!
	//! @param q The queue object.
	//! @param i Index of the queue element, base 0.
	// #define QUEUE_ELEMENT(q, i) (&(q)->m_elements[(q)->m_elementSize * (i)])

	//------------------------------------------------------------------------------
	// Implementation
	//------------------------------------------------------------------------------

	// See ar_kernel.h for documentation of this function.
	Queue::Queue(const char *name, void *storage, std::size_t elementSize, std::size_t capacity) :
		m_elements(reinterpret_cast<uint8_t *>(storage)), m_elementSize(elementSize), m_capacity(capacity), m_runLoopNode(this) {

		if (!assertWrap(storage && elementSize && capacity)) { return; }
		assert(!Port::get_irq_state());

		std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);

#if AR_GLOBAL_OBJECT_LISTS
		queue->m_createdNode.m_obj = queue;
		g_ar_objects.queues.add(&queue->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	// See ar_kernel.h for documentation of this function.
	Queue::~Queue() {
		assert(!Port::get_irq_state());

		// Unblock all threads blocked on this queue.
		while (m_sendBlockedList.m_head) { m_sendBlockedList.getHead<Thread>()->unblockWithStatus(m_sendBlockedList, Status::objectDeletedError); }

		while (m_receiveBlockedList.m_head) { m_receiveBlockedList.getHead<Thread>()->unblockWithStatus(m_receiveBlockedList, Status::objectDeletedError); }

#if AR_GLOBAL_OBJECT_LISTS
		g_ar_objects.queues.remove(&queue->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	Ar::Status Queue::send(const void *element, std::optional<Duration> timeout) {
		if (!element) { return Ar::Status::invalidParameterError; }

		if (Port::get_irq_state()) {
			// only zero timeout allowed from interrupt
			if (timeout.has_value() && timeout.value().isZero()) { return send_isr(element); }
			return Ar::Status::notFromInterruptError;
		}

		{
			KernelLock guard; // TODO: is is really necessary?

			{
				Position wrPos;
				Position wrPosNext;
				do {
					wrPos = m_writePos.load(std::memory_order::relaxed);
					wrPosNext = wrPos;
					wrPosNext.writer = (wrPosNext.writer + 1) % m_capacity;
					if (wrPosNext.writer == m_readPos.load().writer) {
						if (timeout.has_value() && timeout.value().isZero()) { return Status::timeoutError; }

						// Otherwise block until the queue has room.
						auto *thread = Thread::getCurrent();
						thread->block(m_sendBlockedList, timeout);

						// We're back from the scheduler.
						// Check for errors and exit early if there was one.
						if (thread->m_unblockStatus != Ar::Status::success) {
							// Probably timed out waiting for room in the queue.
							m_sendBlockedList.remove(&thread->m_blockedNode);
							return thread->m_unblockStatus;
						}
						continue; // restart wrPos acquire loop
					}
				} while (!m_writePos.compare_exchange_weak(wrPos, wrPosNext));


				// Copy queue element into place.
				auto *elementSlot = QUEUE_ELEMENT(wrPos.writer);

				std::memcpy(elementSlot, element, m_elementSize);
			}

			{
				// this isr was root of nesting
				Position wrPos;
				Position wrPosNext;
				do {
					wrPos = m_writePos.load(std::memory_order::relaxed);
					wrPosNext = wrPos;
					wrPosNext.writerPropagate();
				} while (!m_writePos.compare_exchange_weak(wrPos, wrPosNext));
			}

			send_internal_finalize();

		}
		return Ar::Status::success;
	}

	Ar::Status Queue::send_isr(const void *element) {
		if (!element) { return Ar::Status::invalidParameterError; }

		bool finisher = false;

		{
			Position wrPos;
			Position wrPosNext;
			do {
				wrPos = m_writePos.load(std::memory_order::relaxed);
				wrPosNext = wrPos;
				wrPosNext.writer = (wrPosNext.writer + 1) % m_capacity;
				if (wrPosNext.writer == m_readPos.load().writer) {
					return Status::queueFullError;
				}
			} while (!m_writePos.compare_exchange_weak(wrPos, wrPosNext));

			finisher = wrPos.reader == wrPos.writer; // true if there are no pending unfinished writes (=> we are root of nesting)

			// Copy queue element into place.
			auto *elementSlot = QUEUE_ELEMENT(wrPos.writer);

			std::memcpy(elementSlot, element, m_elementSize);
		}

		if (finisher) {
			// this isr was root of nesting
			Position wrPos;
			Position wrPosNext;
			do {
				wrPos = m_writePos.load(std::memory_order::relaxed);
				wrPosNext = wrPos;
				wrPosNext.writerPropagate();
			} while (!m_writePos.compare_exchange_weak(wrPos, wrPosNext));

			return Kernel::postDeferredAction(Queue::deferred_send_internal_isr_finalize, this);
		}

		return Ar::Status::success;
	}

	void Queue::deferred_send_internal_isr_finalize(void *queue, void *unused) { reinterpret_cast<Queue *>(queue)->send_internal_finalize(); }

	void Queue::send_internal_finalize() {
		KernelLock guard;

		// Are there any threads waiting to receive?
		if (m_receiveBlockedList.m_head) {
			// Unblock the head of the blocked list.
			m_receiveBlockedList.getHead<Thread>()->unblockWithStatus(m_receiveBlockedList, Ar::Status::success);
		}
		// Is the queue associated with a runloop?
		else if (m_runLoop) {
			// Add this queue to the list of pending queues for the runloop, but first check
			// whether the queue is already pending so we don't attempt to add it twice.
			auto &queues = m_runLoop->getQueues();
			if (!queues.contains(&m_runLoopNode)) {
				queues.add(&m_runLoopNode);
				m_runLoop->wake();
			}
		}
	}

	// See ar_kernel.h for documentation of this function.
	Ar::Status Queue::receive(void *element, std::optional<Duration> timeout) {
		if (!element) { return Ar::Status::invalidParameterError; }

		if (Port::get_irq_state()) {
			// only zero timeout allowed from interrupt
			if (timeout.has_value() && timeout.value().isZero()) { return send_isr(element); }
			return Ar::Status::notFromInterruptError;
		}

		{
			KernelLock guard; // TODO: is is really necessary?

			{
				Position rdPos;
				Position rdPosNext;
				do {
					rdPos = m_readPos.load(std::memory_order::relaxed);
					rdPosNext = rdPos;
					rdPosNext.reader = (rdPosNext.reader + 1) % m_capacity;
					if (rdPos.reader == m_writePos.load().reader) {
						if (timeout.has_value() && timeout.value().isZero()) { return Status::timeoutError; }

						// Otherwise block until the queue has new element.
						auto *thread = Thread::getCurrent();
						thread->block(m_receiveBlockedList, timeout);

						// We're back from the scheduler.
						// Check for errors and exit early if there was one.
						if (thread->m_unblockStatus != Ar::Status::success) {
							// Probably timed out waiting for data in the queue.
							m_receiveBlockedList.remove(&thread->m_blockedNode);
							return thread->m_unblockStatus;
						}
						continue; // restart rdPos acquire loop
					}
				} while (!m_readPos.compare_exchange_weak(rdPos, rdPosNext));

				// Copy queue element into place.
				auto *elementSlot = QUEUE_ELEMENT(rdPos.writer);

				std::memcpy(element, elementSlot, m_elementSize);
			}

			{
				// this isr was root of nesting
				Position rdPos;
				Position rdPosNext;
				do {
					rdPos = m_readPos.load(std::memory_order::relaxed);
					rdPosNext = rdPos;
					rdPosNext.readerPropagate();
				} while (!m_readPos.compare_exchange_weak(rdPos, rdPosNext));
			}

			send_internal_finalize();

		}
		return Ar::Status::success;
	}

	Ar::Status Queue::receive_isr(void *element) {
		if (!element) { return Ar::Status::invalidParameterError; }

		bool finisher = false;

		{
			Position rdPos;
			Position rdPosNext;
			do {
				rdPos = m_readPos.load(std::memory_order::relaxed);
				rdPosNext = rdPos;
				rdPosNext.reader = (rdPosNext.reader + 1) % m_capacity;
				if (rdPos.reader == m_writePos.load().reader) {
					return Status::queueFullError;
				}
			} while (!m_readPos.compare_exchange_weak(rdPos, rdPosNext));

			finisher = rdPos.reader == rdPos.writer; // true if there are no pending unfinished writes (=> we are root of nesting)

			// Copy queue element into place.
			auto *elementSlot = QUEUE_ELEMENT(rdPos.reader);

			std::memcpy(element, elementSlot, m_elementSize);
		}

		if (finisher) {
			// this isr was root of nesting
			Position rdPos;
			Position rdPosNext;
			do {
				rdPos = m_readPos.load(std::memory_order::relaxed);
				rdPosNext = rdPos;
				rdPosNext.readerPropagate();
			} while (!m_readPos.compare_exchange_weak(rdPos, rdPosNext));

			return Kernel::postDeferredAction(Queue::deferred_receive_internal_isr_finalize, this);
		}

		return Ar::Status::success;
	}

	void Queue::deferred_receive_internal_isr_finalize(void *queue, void *unused) { reinterpret_cast<Queue *>(queue)->receive_internal_finalize(); }

	void Queue::receive_internal_finalize() {
		KernelLock guard;

		// Are there any threads waiting to send?
		if (m_sendBlockedList.m_head) {
			// Unblock the head of the blocked list.
			auto *thread = m_sendBlockedList.getHead<Thread>();
			thread->unblockWithStatus(m_sendBlockedList, Ar::Status::success);
		}
	}

	void Queue::flush() {
		assert(!Port::get_irq_state()); //not sure if this would work  from isr
		KernelLock guard;
		//move readPos to writepos
		
		Position rdPos;
		Position rdPosNext;
		do {
			rdPos = m_readPos.load(std::memory_order::relaxed);
			rdPosNext = rdPos;
			rdPosNext.reader = m_writePos.load().reader;
		} while (!m_readPos.compare_exchange_weak(rdPos, rdPosNext));
	}
} // namespace Ar