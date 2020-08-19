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

#include "argon/ar_queue.hpp"
#include "argon/ar_thread.hpp"
#include "argon/ar_kernel.hpp"
#include "ar_port.hpp"
#include "ar_assert.hpp"
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

		if (!Ar::assert(storage && elementSize && capacity)) { return; }
		Ar::assert(!Port::get_irq_state());

		std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);

#if AR_GLOBAL_OBJECT_LISTS
		queue->m_createdNode.m_obj = queue;
		g_ar_objects.queues.add(&queue->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	// See ar_kernel.h for documentation of this function.
	Queue::~Queue() {
		Ar::assert(!Port::get_irq_state());

		// Unblock all threads blocked on this queue.
		while (m_sendBlockedList.m_head) { m_sendBlockedList.getHead<Thread>()->unblockWithStatus(m_sendBlockedList, Status::objectDeletedError); }

		while (m_receiveBlockedList.m_head) { m_receiveBlockedList.getHead<Thread>()->unblockWithStatus(m_receiveBlockedList, Status::objectDeletedError); }

#if AR_GLOBAL_OBJECT_LISTS
		g_ar_objects.queues.remove(&queue->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	Ar::Status Queue::send_internal(const void *element, std::optional<Duration> timeout) {
		KernelLock guard;

		// Check for full queue.
		while (m_count >= m_capacity) {
			// If the queue is full and a zero timeout was given, return immediately.
			if (timeout.has_value() && timeout.value()) { return Status::timeoutError; }

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
		}

		// Copy queue element into place.
		auto *elementSlot = QUEUE_ELEMENT(m_tail);

		std::memcpy(elementSlot, element, m_elementSize);

		// Update queue tail pointer and count.
		if (++m_tail >= m_capacity) { m_tail = 0; }
		++m_count;

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

		return Ar::Status::success;
	}

	void Queue::deferred_send(void *object, void *object2) {
		if (!Ar::assert(object)) { return; };
		reinterpret_cast<Queue *>(object)->send_internal(object2, Duration::zero());
	}

	// See ar_kernel.h for documentation of this function.
	Ar::Status Queue::send(const void *element, std::optional<Duration> timeout) {
		if (!element) { return Ar::Status::invalidParameterError; }

		// Handle irq state by deferring the operation.
		if (Port::get_irq_state()) { return Kernel::postDeferredAction(Queue::deferred_send, this, const_cast<void *>(element)); }

		return send_internal(element, timeout);
	}

	// See ar_kernel.h for documentation of this function.
	Ar::Status Queue::receive(void *element, std::optional<Duration> timeout) {
		if (!element) { return Ar::Status::invalidParameterError; }
		Ar::assert(!Port::get_irq_state());

		{
			KernelLock guard;

			// Check for empty queue.
			while (m_count == 0) {
				if (!timeout.value_or(Duration::zero())) { return Status::queueEmptyError; }

				// Otherwise block until the queue has room.
				auto *thread = Thread::getCurrent();
				thread->block(m_receiveBlockedList, timeout);

				// We're back from the scheduler.
				// Check for errors and exit early if there was one.
				if (thread->m_unblockStatus != Ar::Status::success) {
					// Timed out waiting for the queue to not be empty, or another error occurred.
					m_receiveBlockedList.remove(&thread->m_blockedNode);
					return thread->m_unblockStatus;
				}
			}

			// Read out data.
			auto *elementSlot = QUEUE_ELEMENT(m_head);
			std::memcpy(element, elementSlot, m_elementSize);

			// Update queue head and count.
			if (++m_head >= m_capacity) { m_head = 0; }
			--m_count;

			// Are there any threads waiting to send?
			if (m_sendBlockedList.m_head) {
				// Unblock the head of the blocked list.
				auto *thread = m_sendBlockedList.getHead<Thread>();
				thread->unblockWithStatus(m_sendBlockedList, Ar::Status::success);
			}
		}

		return Ar::Status::success;
	}
} // namespace Ar