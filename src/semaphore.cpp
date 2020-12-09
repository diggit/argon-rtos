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
 * @brief Source for Ar microkernel semaphores.
 */
#include "argon/semaphore.hpp"
#include "argon/kernel.hpp"
#include "argon/thread.hpp"
#include "argon/assert.hpp"
#include "argon/port.hpp"
#include <cstring>

namespace Ar {

	//------------------------------------------------------------------------------
	// Prototypes
	//------------------------------------------------------------------------------

	static void ar_semaphore_deferred_get(void *object, void *object2);
	static void ar_semaphore_deferred_put(void *object, void *object2);

	//------------------------------------------------------------------------------
	// Code
	//------------------------------------------------------------------------------

	// See ar_kernel.h for documentation of this function.
	Semaphore::Semaphore(const char *name, unsigned int count) : m_count(count) {
		assert(!Port::get_irq_state());

		std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);

#if AR_GLOBAL_OBJECT_LISTS
		m_createdNode.m_obj = sem;
		g_ar_objects.semaphores.add(&sem->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	// See ar_kernel.h for documentation of this function.
	Semaphore::~Semaphore() {
		assert(!Port::get_irq_state());

		// Unblock all threads blocked on this semaphore.
		while (m_blockedList.m_head) { m_blockedList.getHead<Thread>()->unblockWithStatus(m_blockedList, Status::objectDeletedError); }

#if AR_GLOBAL_OBJECT_LISTS
		g_ar_objects.semaphores.remove(&m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	void Semaphore::deferred_acquire(void *object, void *object2) {
		if (!assertWrap(object)) { return; };
		reinterpret_cast<Semaphore *>(object)->acquire_internal(Duration::zero());
	}

	// See ar_kernel.h for documentation of this function.
	Ar::Status Semaphore::acquire(std::optional<Duration> timeout) {

		// Ensure that only 0 timeouts are specified when called from an IRQ handler.
		if (Port::get_irq_state()) {
			if (timeout.has_value() && timeout.value()) { return Ar::Status::notFromInterruptError; }

			// Handle irq state by deferring the get.
			return Kernel::postDeferredAction(Semaphore::deferred_acquire, this);
		}

		return acquire_internal(timeout);
	}

	Ar::Status Semaphore::acquire_internal(std::optional<Duration> timeout) {
		KernelLock guard;

		while (m_count == 0) {
			// Count is 0, so we must block. Return immediately if the timeout is 0.
			if (timeout.has_value() && timeout.value()) { return Status::timeoutError; }

			// Block this thread on the semaphore.
			Thread *thread = Thread::getCurrent();
			thread->block(m_blockedList, timeout);

			// We're back from the scheduler. We'll loop and recheck the sem counter, in case
			// a higher priority thread grabbed it between when we were unblocked and when we
			// actually started running.

			// Check for errors and exit early if there was one.
			if (thread->m_unblockStatus != Ar::Status::success) {
				// Failed to gain the semaphore, probably due to a timeout.
				m_blockedList.remove(&thread->m_blockedNode);
				return thread->m_unblockStatus;
			}
		}

		// Take ownership of the semaphore.
		assert(m_count > 0);
		--m_count;

		return Ar::Status::success;
	}

	Ar::Status Semaphore::release_internal() {
		KernelLock guard;

		// Increment count.
		++m_count;

		// Are there any threads waiting on this semaphore?
		if (m_blockedList.m_head) {
			// Unblock the head of the blocked list.
			Thread *thread = m_blockedList.getHead<Thread>();
			thread->unblockWithStatus(m_blockedList, Ar::Status::success);
		}

		return Ar::Status::success;
	}

	void Semaphore::deferred_release(void *object, void *object2) {
		if (!assertWrap(object)) { return; };
		reinterpret_cast<Semaphore *>(object)->release_internal();
	}

	// See ar_kernel.h for documentation of this function.
	Ar::Status Semaphore::release() {
		// Handle irq state by deferring the put.
		if (Port::get_irq_state()) { return Kernel::postDeferredAction(Semaphore::deferred_release, this); }

		return release_internal();
	}

} // namespace Ar

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
