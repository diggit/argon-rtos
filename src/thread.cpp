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
 * @brief Source for Ar threads.
 */

#include "argon/common.hpp"
#include "trace.hpp"
#include "argon/assert.hpp"
#include "argon/port.hpp"
#include "argon/kernel.hpp"
#include "argon/thread.hpp"
#include "argon/runloop.hpp"
#include <cstring>
#include <cstdio>

namespace Ar {

	//------------------------------------------------------------------------------
	// Code
	//------------------------------------------------------------------------------

	// See ar_kernel.h for documentation of this function.
	Thread::Thread(const char *name, Entry entry, void *param, void *stack, std::size_t stackSize, Priority priority, InitialState initialState) :
		m_stackBottom(reinterpret_cast<std::uint32_t *>(stack)),
		m_priority(priority),
		m_entry(thread_entry),
		m_threadNode(this),
		m_blockedNode(this),
		m_uniqueId(Kernel::getNewThreadId()),
		m_userEntry(entry) {
		// Assertions.
		assert(priority >= minThreadPriority || (entry == Kernel::idle_entry && priority == idleThreadPriority));
		assert(stackSize >= sizeof(Port::ThreadContext));
		assert(!Port::get_irq_state());

		if (!stack) {
			m_allocatedStack = new std::uint8_t[stackSize];
			if (!assertWrap(m_allocatedStack)) { return; }
			stack = static_cast<void *>(m_allocatedStack);
		} else {
			m_allocatedStack = nullptr;
		}

		std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);

#if AR_GLOBAL_OBJECT_LISTS
		thread->m_createdNode.m_obj = this;
		g_ar_objects.threads.add(&thread->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS

		// prepare top of stack
		Port::prepare_stack(this, stackSize, param);

		{
			// disable interrupts
			KernelLock guard;

			// add to suspended list
			Kernel::suspendedList().add(&m_threadNode);
		}

		Trace::trace_2(Trace::Event::CREATED, 0, this);

		// Resume thread if requested.
		if (initialState == InitialState::running) { resume(); }
	}

	// See ar_kernel.h for documentation of this function.
	Thread::~Thread() {
		assert(!Port::get_irq_state());

		// Clear runloop association.
		if (m_runLoop) {
			m_runLoop->m_thread = nullptr;
			m_runLoop = nullptr;
		}

#if AR_GLOBAL_OBJECT_LISTS
		g_ar_objects.threads.remove(&m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS

		Trace::trace_2(Trace::Event::DELETED, 0, this);

		// Remove from whatever list the thread is on, and set state to done.
		// If we're deleting the current thread, then execution will never proceed past
		// this block.
		{
			KernelLock guard;

			switch (m_state) {
				case ThreadState::ready:
				case ThreadState::running:
					Kernel::readyList().remove(&m_threadNode);
					Kernel::update_round_robin();
					break;

				case ThreadState::suspended: Kernel::suspendedList().remove(&m_threadNode); break;

				case ThreadState::blocked:
				case ThreadState::sleeping: Kernel::sleepingList().remove(&m_threadNode); break;

				case ThreadState::done: break;

				case ThreadState::unknown: assert(false); return;
			}

			// Mark thread as finished.
			m_state = ThreadState::done;

			// Are we deleting ourself?
			if (this == Thread::getCurrent()) { Kernel::flags().needsReschedule = true; }
		}

		// Free dynamically allocated stack.
		if (m_allocatedStack) { delete[] m_allocatedStack; }
	}

	StatusW Thread::resume_internal() {
		KernelLock guard;

		switch (m_state) {
			case ThreadState::ready:
			case ThreadState::running: return Ar::Status::success;

			case ThreadState::suspended: Kernel::suspendedList().remove(&m_threadNode); break;

			case ThreadState::sleeping: Kernel::sleepingList().remove(&m_threadNode); break;

			case ThreadState::blocked:
			case ThreadState::unknown:
			case ThreadState::done: return Status::invalidStateError;
		}

		// Put the thread back on the ready list.
		m_state = ThreadState::ready;
		Kernel::readyList().add(&m_threadNode);
		Kernel::update_round_robin();

		// yield to scheduler if there is not a running thread or if this thread
		// has a higher priority that the running one
		if (m_priority > Thread::getCurrent()->m_priority) { Kernel::flags().needsReschedule = true; }

		return Ar::Status::success;
	}

	void Thread::deferred_resume(void *object, void *object2) {
		if (!assertWrap(object)) { return; }
		reinterpret_cast<Thread *>(object)->resume_internal();
	}

	// See ar_kernel.h for documentation of this function.
	StatusW Thread::resume() {

		// Check thread state.
		switch (m_state) {
			// Nothing to do if the thread is already ready.
			case ThreadState::ready:
			case ThreadState::running: return Ar::Status::success;

			// These are states from which we can resume the thread.
			case ThreadState::suspended:
			case ThreadState::sleeping: break;

			// Erroneous states.
			case ThreadState::blocked:
			case ThreadState::unknown:
			case ThreadState::done: return Status::invalidStateError;
		}

		if (Port::get_irq_state()) {
			// Handle irq state by deferring the resume.
			return Kernel::postDeferredAction(Thread::deferred_resume, this);
		} else {
			return resume_internal();
		}
	}

	StatusW Thread::suspend_internal() {
		KernelLock guard;

		switch (m_state) {
			// Move ready threads to suspended list.
			case ThreadState::ready:
			case ThreadState::running:
				Kernel::readyList().remove(&m_threadNode);
				Kernel::update_round_robin();
				break;

			// Move sleeping threads to suspended list.
			case ThreadState::sleeping: Kernel::sleepingList().remove(&m_threadNode); break;

			// Nothing needs doing if the thread is already suspended.
			case ThreadState::suspended: return Ar::Status::success;

			// Erroneous states.
			case ThreadState::blocked:
			case ThreadState::unknown:
			case ThreadState::done: return Status::invalidStateError;
		}

		// Move the thread to the suspended list.
		m_state = ThreadState::suspended;
		Kernel::suspendedList().add(&m_threadNode);

		// are we suspending the current thread?
		if (this == Thread::getCurrent()) { Kernel::flags().needsReschedule = true; }

		return Ar::Status::success;
	}

	void Thread::deferred_suspend(void *object, void *object2) {
		if (!assertWrap(object)) { return; }
		reinterpret_cast<Thread *>(object)->suspend_internal();
	}

	// See ar_kernel.h for documentation of this function.
	StatusW Thread::suspend() {

		// Check thread state.
		switch (m_state) {
			// Nothing to do if the thread is already ready.
			case ThreadState::suspended: return Ar::Status::success;

			// These are states from which we can suspend the thread.
			case ThreadState::ready:
			case ThreadState::running:
			case ThreadState::sleeping: break;

			// Erroneous states.
			case ThreadState::blocked: // Would have to remove thread from the blocked list of the kernel object.
			case ThreadState::unknown:
			case ThreadState::done: return Status::invalidStateError;
		}

		if (Port::get_irq_state()) {
			// Handle irq state by deferring the resume.
			return Kernel::postDeferredAction(Thread::deferred_suspend, this);
		} else {
			return suspend_internal();
		}
	}

	// See ar_kernel.h for documentation of this function.
	StatusW Thread::setPriority(Priority priority) {
		if (Port::get_irq_state()) { return Ar::Status::notFromInterruptError; }

		if (priority == Thread::idleThreadPriority && this != &Kernel::idleThread()) { return Status::invalidPriorityError; }

		if (priority != m_priority) {
			KernelLock guard;

			// Set new priority.
			m_priority = priority;

			// Resort ready list.
			if (m_state == ThreadState::ready || m_state == ThreadState::running) {
				Kernel::readyList().remove(&m_threadNode);
				Kernel::readyList().add(&m_threadNode);
				Kernel::update_round_robin();
			} else if (m_state == ThreadState::blocked) {
				//! @todo Resort blocked list and handle priority inheritance.
			}

			Kernel::flags().needsReschedule = true;
		}

		return Ar::Status::success;
	}

	// See ar_kernel.h for documentation of this function.
	void Thread::sleep(std::optional<Duration> duration) {
		auto now = Time::now();
		// Cannot sleep in interrupt context.
		if (Port::get_irq_state()) { return; }

		auto thread = Thread::getCurrent();

		// bail if there is not a running thread to put to sleep
		if ((duration.has_value() && duration.value().isZero()) || !thread) { return; }

		// Sleeping infinitely is equivalent to suspending the thread.
		if (!duration.has_value()) {
			thread->suspend();
		} else {
			sleepUntil(now + duration.value());
		}
	}

	// See ar_kernel.h for documentation of this function.
	void Thread::sleepUntil(Time time) {
		// Cannot sleep in interrupt context.
		if (Port::get_irq_state()) { return; }

		auto thread = Thread::getCurrent();

		// bail if there is not a running thread to put to sleep
		if (time <= Time::now() || !thread) { return; }

		{
			KernelLock guard;

			// put the current thread on the sleeping list
			thread->m_wakeupTime = time;

			Kernel::readyList().remove(&thread->m_threadNode);
			thread->m_state = ThreadState::sleeping;
			Kernel::sleepingList().add(&thread->m_threadNode);
			Kernel::update_round_robin();

			// run scheduler and switch to another thread
			Kernel::flags().needsReschedule = true;
		}
	}

//! The thread wrapper calls the thread entry function that was set in
//! the init() call. When and if the function returns, the thread is removed
//! from the ready list and its state set to #ThreadState::done.
//!
//! This function will never actually itself return. Instead, it switches to
//! the scheduler before exiting, and the scheduler will never switch back
//! because the thread is marked as done.
//!
//! @param thread Pointer to the thread object which is starting up.
//! @param param Arbitrary parameter passed to the thread entry point.
#if !defined(__ICCARM__)
	__attribute__((noreturn))
#endif
	void Thread::wrapper(Thread *thread, void *param) {
		assert(thread);

		// Call the entry point.
		if (thread->m_entry) { thread->m_entry(param); }

		// Thread function has finished, so clean up and terminate the thread.
		{
			KernelLock guard;

			// This thread must be in the running state for this code to execute,
			// so we know it is on the ready list.
			Kernel::readyList().remove(&thread->m_threadNode);
			Kernel::update_round_robin();

			// Mark this thread as finished
			thread->m_state = ThreadState::done;
		}

		// Switch to the scheduler to let another thread take over
		Kernel::enter_scheduler();

		// The compiler will see this and know we're not returning.
		for (;;) {}
	}

	//! The thread is removed from the ready list. It is placed on the blocked list
	//! referenced by the @a blockedList argument and its state is set to
	//! #ThreadState::blocked. If the timeout is non-infinite, the thread is also
	//! placed on the sleeping list with the wakeup time set to when the timeout
	//! expires.
	//!
	//! @param[in,out] blockedList Reference to the head of the linked list of
	//!     blocked threads.
	//! @param timeout The maximum number of ticks that the thread can remain
	//!     blocked. A value of #kArInfiniteTimeout means the thread can be
	//!     blocked forever. A timeout of 0 is not allowed and should be handled
	//!     by the caller.
	void Thread::block(List &blockedList, std::optional<Duration> timeout) {
		assert(!timeout.has_value() || !timeout.value().isZero());

		// Remove this thread from the ready list.
		Kernel::readyList().remove(&m_threadNode);
		Kernel::update_round_robin();

		// Update its state.
		m_state = ThreadState::blocked;
		m_unblockStatus = Ar::Status::success;

		// Add to blocked list.
		blockedList.add(&m_blockedNode);

		// If a valid timeout was given, put the thread on the sleeping list.
		if (timeout.has_value()) {
			m_wakeupTime = Time::now() + timeout.value();
			Kernel::sleepingList().add(&m_threadNode);
		} else {
			// Signal when unblocking that this thread is not on the sleeping list.
			m_wakeupTime.reset();
		}

		// Enter scheduler now that this thread is blocked.
		{
			KernelUnlock guard;

			// Yield to the scheduler. We'll return when the thread is unblocked by another thread.
			Kernel::enter_scheduler();
		}
	}

	//! If the thread had a valid timeout when it was blocked, it is removed from
	//! the sleeping list. It is always removed from the blocked list passed in
	//! @a blockedList. And finally the thread is restored to ready status by setting
	//! its state to #ThreadState::ready and adding it back to the ready list.
	//!
	//! @param[in,out] blockedList Reference to the head of the linked list of
	//!     blocked threads.
	//! @param unblockStatus Status code to return from the function that
	//!     the thread had called when it was originally blocked.
	void Thread::unblockWithStatus(List &blockedList, Ar::Status unblockStatus) {
		assert(m_state == ThreadState::blocked);

		// Remove from the sleeping list if it was on there. Won't hurt if
		// the thread is not on that list.
		if (m_wakeupTime && Kernel::sleepingList().m_head) { Kernel::sleepingList().remove(&m_threadNode); }

		// Remove this thread from the blocked list.
		blockedList.remove(&m_blockedNode);

		// Put the unblocked thread back onto the ready list.
		m_state = ThreadState::ready;
		m_unblockStatus = unblockStatus;
		Kernel::readyList().add(&m_threadNode);
		Kernel::update_round_robin();

		// Invoke the scheduler if the unblocked thread is higher priority than the current one.
		if (m_priority > Thread::getCurrent()->m_priority) { Kernel::flags().needsReschedule = true; }
	}

	// See ar_kernel.h for documentation of this function.
	Thread *ar_thread_get_current(void) { return Thread::getCurrent(); }

	std::uint16_t Thread::get_load(Duration measurementTime) { return Kernel::get_thread_load(this); }

	// See ar_kernel.h for documentation of this function.
	std::size_t Thread::getStackUsedNow() {
		// Check canary.
		if (*m_stackBottom != Port::stackCheckValue) { return 0; }
		std::uint32_t stackPointer {};
		if (this == getCurrent()) {
			stackPointer = __get_PSP();
		} else {
			stackPointer = reinterpret_cast<std::uint32_t>(m_stackPointer);
		}
		return reinterpret_cast<std::uint32_t>(m_stackTop) - stackPointer;
	}

	std::size_t Thread::getStackUsedMax() {
		// Check canary.
		if (*m_stackBottom != Port::stackCheckValue) { return 0; }
		std::uint32_t *stack = m_stackBottom + 1; // Skip canary.
		std::uint32_t unusedWords = 1; // Count canary.
		// Handle part of stack being used to hold member function thread entry points.
		if (*(stack + 2) == Port::stackFillValue) {
			unusedWords += 2;
			stack += 2;
		}
		for (; stack < m_stackTop; ++stack, ++unusedWords) {
			if (*stack != Port::stackFillValue) { break; }
		}
		std::uint32_t stackSize = reinterpret_cast<std::uint32_t>(m_stackTop) - reinterpret_cast<std::uint32_t>(m_stackBottom);
		return stackSize - (unusedWords * sizeof(std::uint32_t));
	}

	// See ar_kernel.h for documentation of this function.
	std::size_t Thread::getReport(ThreadStatus *report, std::size_t maxEntries, Duration loadMeasurementTime) {
		std::size_t threadCount = 0;
#if AR_GLOBAL_OBJECT_LISTS
		std::array<List *const, 1> threadLists = {&g_ar_objects.threads};
#else
		std::array<List *const, 3> threadLists = {&Kernel::readyList(), &Kernel::suspendedList(), &Kernel::sleepingList()};
#endif // AR_GLOBAL_OBJECT_LISTS

		for (std::uint32_t i = 0; i < std::size(threadLists) && threadCount < maxEntries; ++i) {
			auto start = threadLists[i]->m_head;
			if (start) {
				auto node = start;
				do {
					auto *thread = node->getObject<Thread>();

					if (report) {
						*report = ThreadStatus(thread, thread->m_name, thread->m_uniqueId, Kernel::get_thread_load(thread), thread->m_state,
							thread->getStackUsedMax(),
							reinterpret_cast<std::size_t>(thread->m_stackTop) - reinterpret_cast<std::size_t>(thread->m_stackBottom));
						++report;
					}
					++threadCount;

					node = node->m_next;
				} while (node != start && threadCount < maxEntries);
			}
		}

		return threadCount;
	}

	// See ar_classes.h for documentation of this function.
	void Thread::threadEntry(void *param) {
		if (m_userEntry) { m_userEntry(param); }
	}

	// See ar_classes.h for documentation of this function.
	void Thread::thread_entry(void *param) {
		Thread *thread = getCurrent();
		assert(thread);
		thread->threadEntry(param);
	}

} // namespace Ar

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
