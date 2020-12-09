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
 * @brief Source for Ar microkernel mutexes.
 */

#include "argon/kernel.hpp"
#include "argon/mutex.hpp"
#include "argon/thread.hpp"
#include "argon/common.hpp"
#include "argon/assert.hpp"
#include <cstring>

namespace Ar {

	//------------------------------------------------------------------------------
	// Prototypes
	//------------------------------------------------------------------------------

	//------------------------------------------------------------------------------
	// Implementation
	//------------------------------------------------------------------------------

	// See ar_kernel.h for documentation of this function.
	Mutex::Mutex(const char *name) {
		assert(!Port::get_irq_state());

		std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);
		;

#if AR_GLOBAL_OBJECT_LISTS
		m_createdNode.m_obj = mutex;
		g_ar_objects.mutexes.add(&m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	// See ar_kernel.h for documentation of this function.
	//! @todo Return error when deleting a mutex that is still locked.
	Mutex::~Mutex() {
		assert(!Port::get_irq_state());

#if AR_GLOBAL_OBJECT_LISTS
		g_ar_objects.mutexes.remove(&mutex->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	void Mutex::deferred_lock(void *object, void *object2) {
		if (!assertWrap(object)) { return; };
		reinterpret_cast<Mutex *>(object)->lock(Duration::zero());
	}

	// See ar_kernel.h for documentation of this function.
	StatusW Mutex::lock(std::optional<Duration> timeout) {
		assert(!Port::get_irq_state());

		// Handle irq state by deferring the get.
		if (Port::get_irq_state()) { return Kernel::postDeferredAction(Mutex::deferred_lock, this); }

		return lock_internal();
	}

	StatusW Mutex::lock_internal(std::optional<Duration> timeout) {
		KernelLock guard;

		// If this thread already owns the mutex, just increment the count.
		if (Thread::getCurrent() == m_owner) {
			m_ownerLockCount = m_ownerLockCount + 1;
		}
		// Otherwise attempt to get the mutex.
		else {
			// Will we block?
			while (m_ownerLockCount != 0) {
				// Return immediately if the timeout is 0.
				if (timeout.has_value() && timeout.value()) { return Status::timeoutError; }

				// Check if we need to hoist the owning thread's priority to our own.
				Thread *self = Thread::getCurrent();
				assert(m_owner);
				if (self->m_priority > m_owner->m_priority) {
					if (!m_originalPriority) { m_originalPriority = m_owner->m_priority; }
					m_owner->m_priority = self->m_priority;
				}

				// Block this thread on the mutex.
				self->block(m_blockedList, timeout);

				// We're back from the scheduler. We'll loop and recheck the ownership counter, in case
				// a higher priority thread grabbed the lock between when we were unblocked and when we
				// actually started running.

				// Check for errors and exit early if there was one.
				if (self->m_unblockStatus != Ar::Status::success) {
					//! @todo Need to handle timeout after hoisting the owner thread.
					// Failed to gain the mutex, probably due to a timeout.
					m_blockedList.remove(&self->m_blockedNode);
					return self->m_unblockStatus;
				}
			}

			// Take ownership of the lock.
			assert(m_owner == nullptr && m_ownerLockCount == 0);
			m_owner = Thread::getCurrent();
			m_ownerLockCount = m_ownerLockCount + 1;
		}

		return Ar::Status::success;
	}

	void Mutex::deferred_unlock(void *object, void *object2) {
		if (!assertWrap(object)) { return; };
		reinterpret_cast<Mutex *>(object)->unlock_internal();
	}

	// See ar_kernel.h for documentation of this function.
	StatusW Mutex::unlock() {

		// Handle irq state by deferring the put.
		if (Port::get_irq_state()) { return Kernel::postDeferredAction(Mutex::deferred_unlock, this); }

		return unlock_internal();
	}

	StatusW Mutex::unlock_internal() {
		KernelLock guard;

		// Nothing to do if the mutex is already unlocked.
		if (m_ownerLockCount == 0) { return Status::alreadyUnlockedError; }

		// Only the owning thread can unlock a mutex.
		Thread *self = Thread::getCurrent();
		if (self != m_owner) { return Status::notOwnerError; }

		// We are the owner of the mutex, so decrement its recursive lock count.
		m_ownerLockCount = m_ownerLockCount - 1;
		if (m_ownerLockCount == 0) {
			// The lock count has reached zero, so clear the owner.
			m_owner = nullptr;

			// Restore this thread's priority if it had been raised.
			uint8_t original = m_originalPriority;
			if (original) {
				m_originalPriority = 0;
				self->setPriority(original);
			}

			// Unblock a waiting thread.
			if (m_blockedList.m_head) {
				// Unblock the head of the blocked list.
				Thread *thread = m_blockedList.getHead<Thread>();
				thread->unblockWithStatus(m_blockedList, Ar::Status::success);
			}
		}

		return Ar::Status::success;
	}
}