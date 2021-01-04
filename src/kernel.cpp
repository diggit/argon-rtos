/*
 * Copyright (c) 2007-2020 Immo Software
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
 * @file ar_kernel.cpp
 * @brief Source for Ar kernel.
 */

#include <cstdint>
#include <cstring>
#include "trace.hpp"
#include "argon/kernel.hpp"
#include "argon/thread.hpp"
#include "argon/assert.hpp"

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

namespace Ar {

	//------------------------------------------------------------------------------
	// Variables
	//------------------------------------------------------------------------------

	//! Global kernel state.
	//!
	//! @internal
	//! The initializer for this struct sets the readyList and sleepingList sort
	//! predicates so that those lists will be properly sorted when populated by
	//! threads created through static initialization. Although static initialization
	//! order is not guaranteed, we can be sure that initialization of `g_ar` from
	//! .rodata will happen before static initializers are called.

	Kernel &kernel() {
		static Kernel _kernel;
		return _kernel;
	}

#if AR_GLOBAL_OBJECT_LISTS
	//! Global list of kernel objects.
	ar_all_objects_t g_ar_objects = {0};
#endif // AR_GLOBAL_OBJECT_LISTS

	//! The stack for the idle thread.
	std::array<std::uint8_t, config::IDLE_THREAD_STACK_SIZE> s_idleThreadStack;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

//! @brief System idle thread entry point.
//!
//! This thread just spins forever.
//!
//! @param param Ignored.
#if !defined(__ICCARM__)
	__attribute__((noreturn))
#endif
	void Kernel::idle_entry(void *param) {
		for (;;) {
			// while (kernel().m_nextWakeup.has_value() && Time::now() < kernel().m_nextWakeup.value()) {}

			if constexpr (config::ENABLE_IDLE_SLEEP) {
				__DSB();
				// Hitting this bit puts the processor to sleep until the next interrupt fires.
				__WFI();
				__ISB();
			}
		}
	}

	//! Cause the scheduler to be run.
	//!
	//! If this function is called when the kernel is locked, a flag is set that
	//! will cause the scheduler to be entered immediately upon the kernel being
	//! unlocked.
	void Kernel::enter_scheduler_internal() {
		// Do nothing if kernel isn't running yet.
		if (!m_flags.isRunning) { return; }

		if (!m_lockCount) {
			// Clear rescheduler.
			m_flags.needsReschedule = false;

			// Call port-specific function to invoke the scheduler.
			Port::service_call();
		} else {
			m_flags.needsReschedule = true;
		}
	}

	// See ar_kernel.h for documentation of this function.
	void Kernel::run_internal() {
		// Assert if there is no thread ready to run.
		assert(m_readyList.m_head);

		// Init some misc fields.
		// Note: we do _not_ init threadIdCounter since it will already have been incremented
		// some for any statically-initialized threads.
		// Note: list predicates were initialized by the g_ar initializer.

		// Create the idle thread. Priority 1 is passed to init function to pass the
		// assertion and then set to the correct 0 manually.
		// ar_thread_create(&idleThread, "idle", idle_entry, 0, s_idleThreadStack, sizeof(s_idleThreadStack), 1, kArSuspendThread);
		Kernel::idleThread().m_priority = 0;
		// ar_thread_resume(&idleThread);
		Kernel::idleThread().resume();

		// Set up system tick timer
		Port::init_timer();

		// Init port.
		Port::init_system();

		// Init trace.
		Trace::trace_init();

		// We're now ready to run
		m_flags.isRunning = true;

		// Enter into the scheduler. The yieldIsr() will see that s_currentThread
		// is nullptr and ignore the stack pointer it was given. After the scheduler
		// runs, we return from the scheduler to a ready thread.
		enter_scheduler();

		// should never reach here
		Port::halt();
	}

	void Kernel::timerIsr_internal() {
		// Exit immediately if the kernel isn't running.
		if (!m_flags.isRunning) { return; }

		// If the kernel is locked, record that we missed this tick and come back as soon
		// as the kernel gets unlocked.
		if (m_lockCount) { return; }

		// Process elapsed time. Invoke the scheduler if any threads were woken or if
		// round robin scheduling is in effect.
		if (process_ticks(true) || m_flags.needsRoundRobin) { Port::service_call(); }
	}

	//! @param topOfStack This parameter should be the stack pointer of the thread that was
	//!     current when the timer IRQ fired.
	//! @return The value of the current thread's stack pointer is returned. If the scheduler
	//!     changed the current thread, this will be a different value from what was passed
	//!     in @a topOfStack.
	std::uint32_t Kernel::yieldIsr_internal(std::uint32_t topOfStack) {
		assert(!m_lockCount);

		// save top of stack for the thread we interrupted
		if (m_currentThread) { m_currentThread->m_stackPointer = reinterpret_cast<uint8_t *>(topOfStack); }

		// Process any deferred actions.
		run_deferred_actions();

		process_ticks(false);

		// Run the scheduler. It will modify currentThread if switching threads.
		scheduler();
		m_flags.needsReschedule = false;

		// The idle thread prevents this condition.
		assert(m_currentThread);

		// return the new thread's stack pointer
		return reinterpret_cast<std::uint32_t>(m_currentThread->m_stackPointer);
	}

	//! Increments the system tick count and wakes any sleeping threads whose wakeup time
	//! has arrived. If the thread's state is #kArThreadBlocked then its unblock status
	//! is set to #kArTimeoutError.
	//!
	//! @param ticks The number of ticks that have elapsed. Normally this will only be 1,
	//!     and must be at least 1, but may be higher if interrupts are disabled for a
	//!     long time.
	//! @return Flag indicating whether any threads were modified.
	bool Kernel::process_ticks(bool fromTimerIrq) {

		// Compare against next wakeup time we previously computed.
		if (!m_nextWakeup.has_value()) { return false; }
		if (Time::now() < kernel().m_nextWakeup.value()) {
			assert(!fromTimerIrq);
			return false;
		}

		// Scan list of sleeping threads to see if any should wake up.
		List::Node *node = kernel().m_sleepingList.m_head;
		bool wasThreadWoken = false;

		if (node) {
			do {
				auto *thread = node->getObject<Thread>();
				auto *next = node->m_next;

				// Is it time to wake this thread?
				// TODO: would it be safe to cache time value?
				if (thread->m_wakeupTime.has_value() && Time::now() >= thread->m_wakeupTime.value()) {
					wasThreadWoken = true;

					// State-specific actions
					switch (thread->m_state) {
						case Thread::ThreadState::sleeping:
							// The thread was just sleeping.
							break;

						case Thread::ThreadState::blocked:
							// The thread has timed out waiting for a resource.
							thread->m_unblockStatus = Status::timeoutError;
							break;

						default:
							// Should not have threads in other states on this list!
							Port::halt();
					}

					// Put thread in ready state.
					m_sleepingList.remove(&thread->m_threadNode);
					thread->m_state = Thread::ThreadState::ready;
					m_readyList.add(&thread->m_threadNode);
					Kernel::update_round_robin();
				}
				// Exit loop if we hit a thread with a wakeup time in the future. The sleeping list
				// is sorted, so there will be no further threads to handle in the list.
				else {
					break;
				}

				node = next;
			} while (m_sleepingList.m_head && node != m_sleepingList.m_head);
		}

		return wasThreadWoken;
	}

	//! @brief Execute actions deferred from interrupt context.
	void Kernel::run_deferred_actions() {
		// Kernel must not be locked. However, executing deferred actions will temporarily
		// lock the kernel below.
		assert(m_lockCount == 0);

		m_flags.isRunningDeferred = true;

		// Pull actions from the head of the queue and execute them.
		auto &queue = m_deferredActions;
		auto i = queue.m_first.load();
		while (!queue.isEmpty()) {
			std::int32_t iPlusOne = i + 1;
			if (iPlusOne >= config::DEFERRED_ACTION_QUEUE_SIZE) { iPlusOne = 0; }

			auto &entry = queue.m_entries[i];

			// Ignore action entries that contain an extra argument value for the previous action.
			if (reinterpret_cast<uint32_t>(entry.action) != DeferredActionQueue::actionExtraValue) {
				assert(entry.action);
				entry.action(entry.object, queue.m_entries[iPlusOne].object);
			}

			// Atomically remove the entry we just processed from the queue.
			// This is the only code that modifies the m_first member of the queue.
			i = iPlusOne;
			queue.m_count--;
			queue.m_first = i;
		}

		m_flags.isRunningDeferred = false;
	}

	//! @brief Function to make it clear what happened.
	void THREAD_STACK_OVERFLOW_DETECTED() { Port::halt(); }

	void Kernel::scheduler() {
		// There must always be at least one thread on the ready list.
		assert(kernel().m_readyList.m_head);

		auto schedulerEntryTime = Time::now();

		// Find the next ready thread.
		auto *firstNode = m_readyList.m_head;
		auto *first = firstNode->getObject<Thread>();
		Thread *highest = nullptr;

		// Handle these cases by selecting the first thread in the ready list, which will have the
		// highest priority since the ready list is sorted.
		// 1. The first time the scheduler runs and kernel().currentThread is nullptr.
		// 2. The current thread was suspended.
		// 3. Higher priority thread became ready.
		if (!m_currentThread || m_currentThread->m_state != Thread::ThreadState::running || first->m_priority > m_currentThread->m_priority) {
			highest = first;
		}
		// Else handle these cases:
		// 2. We're performing round-robin scheduling.
		// 3. Shouldn't switch the thread.
		else {
			// Start with the current thread.
			auto *startNode = &m_currentThread->m_threadNode;
			const auto startPriority = startNode->getObject<Thread>()->m_priority;

			// Pick up the next thread in the ready list.
			auto *nextNode = startNode->m_next;
			highest = nextNode->getObject<Thread>();

			// If the next thread is not the same priority, then go back to the start of the ready list.
			if (highest->m_priority != startPriority) {
				highest = first;
				assert(highest->m_priority == startPriority);
			}
		}

		// Switch to newly selected thread.
		assert(highest);
		if (highest != m_currentThread) {
			if (m_currentThread && m_currentThread->m_state == Thread::ThreadState::running) { m_currentThread->m_state = Thread::ThreadState::ready; }

			Trace::trace_1(Trace::Event::SWITCHED, (to_underlying(m_currentThread->m_state) << 16) | highest->m_uniqueId);

			highest->m_state = Thread::ThreadState::running;
			if constexpr (config::ENABLE_SYSTEM_LOAD || config::ENABLE_SYSTEM_LOAD_PER_THREAD) {
				const auto timeSpentInThread = schedulerEntryTime - systemLoad.getLastSwitchIn();

				if constexpr (config::ENABLE_SYSTEM_LOAD_PER_THREAD) {
					if (m_currentThread) {
						m_currentThread->systemLoad.accumulate(timeSpentInThread); // accumulate active time of thread
					}
				}

				systemLoad.recordSwitch(); // TODO: maybe too early
				if (m_currentThread && m_currentThread != &Kernel::idleThread()) { // accumulate times only from regular threads
					systemLoad.accumulate(timeSpentInThread);
				}
			}
			auto prevThread = m_currentThread;
			m_currentThread = highest;
			if constexpr (Ar::config::REFILL_UNUSED_STACK_ON_SUSPEND) {
				Port::refill_stack(prevThread);
			}
		}

		// Check for stack overflow on the current thread.
		if (m_currentThread) {
			const auto current = reinterpret_cast<std::uintptr_t>(m_currentThread->m_stackPointer);
			const auto bottom = reinterpret_cast<std::uintptr_t>(m_currentThread->m_stackBottom);
			const auto check = *(m_currentThread->m_stackBottom);
			if (!assertWrap((current >= bottom) && (check == Port::stackCheckValue))) { THREAD_STACK_OVERFLOW_DETECTED(); }
		}

		// Compute delay until next wakeup event and adjust timer.
		auto wakeup = get_next_wakeup_time();
		if (m_nextWakeup.has_value() != wakeup.has_value() || (m_nextWakeup.has_value() && wakeup.has_value() && (wakeup.value() != m_nextWakeup.value()))) {
			m_nextWakeup = wakeup;
			Port::set_wakeup_time(m_nextWakeup);
		}
	}

	//! @brief Cache whether round-robin scheduling needs to be used.
	//!
	//! Round-robin is required if there are multiple ready threads with the same priority. Since the
	//! ready list is sorted by priority, we can just check the first two nodes to see if they are the
	//! same priority.
	void Kernel::update_round_robin_internal() {
		List::Node *node = m_readyList.m_head;
		assert(node);
		auto pri1 = node->getObject<Thread>()->m_priority;
		if (node->m_next != node) {
			node = node->m_next;
			auto pri2 = node->getObject<Thread>()->m_priority;

			m_flags.needsRoundRobin = (pri1 == pri2);
		} else {
			m_flags.needsRoundRobin = false;
		}
	}

	//! @brief Determine the delay to the next wakeup event.
	//!
	//! Wakeup events are either sleeping threads that are scheduled to wake, or a timer that is
	//! scheduled to fire.
	//!
	//! @return The number of ticks until the next wakup event. If the result is 0, then there are no
	//!     wakeup events pending.
	std::optional<Time> Kernel::get_next_wakeup_time() {
		std::optional<Time> wakeup {};

		// See if round-robin needs to be used.
		if (m_flags.needsRoundRobin) {
			// No need to check sleeping threads!
			return Time::now() + config::schedulerPeriod;
		}

		// Check for a sleeping thread. The sleeping list is sorted by wakeup time, so we only
		// need to look at the list head.
		auto node = m_sleepingList.m_head;
		if (node) {
			auto *thread = node->getObject<Thread>();
			auto threadWakeup = thread->m_wakeupTime;
			if (threadWakeup && (!wakeup || threadWakeup.value() < wakeup.value())) { wakeup = threadWakeup; }
		}

		return wakeup;
	}

	// See ar_kernel.h for documentation of this function.
	std::uint16_t Kernel::get_system_load_internal(bool reset) {
		if constexpr (config::ENABLE_SYSTEM_LOAD || config::ENABLE_SYSTEM_LOAD_PER_THREAD) {
			const auto now = Time::now();
			const auto measurementPeriod = now - systemLoad.getLoadtStart();
			const auto accumulatedTotal = systemLoad.getAccumulated() + (now - systemLoad.getLastSwitchIn()); // because this thread is running and its' busy time was not accumulated yet

			if constexpr (__FPU_USED) {
				auto result = (accumulatedTotal) / measurementPeriod * 1000;
				if (reset) { reset_system_load_internal(); }
				return result;
			} else {
				auto result = (accumulatedTotal).in<std::micro>() * 1000 / measurementPeriod.in<std::micro>(); // FPU less version, may overflow in extreme cases
				if (reset) { reset_system_load_internal(); }
				return result;
			}

		} else {
			return 0;
		}
	}

	std::uint16_t Kernel::get_thread_load_internal(Thread *thread) {
		if constexpr (config::ENABLE_SYSTEM_LOAD_PER_THREAD) {
			if (!thread) { return 0; }
			const auto now = Time::now();
			const auto measurementPeriod = now - systemLoad.getLoadtStart();
			
			auto busyTime = thread->systemLoad.getAccumulator();
			if (thread == Kernel::getCurrent()) {
				//requested thread is still running and time since it switch in was not accumulated yet. add it manually
				busyTime += now - systemLoad.getLastSwitchIn();
			}
			if constexpr (__FPU_USED) {
				return busyTime / measurementPeriod * 1000;
			} else {
				return busyTime.in<std::micro>() * 1000 / measurementPeriod.in<std::micro>();
			}
		} else {
			return 0;
		}
	}

	void Kernel::reset_system_load_internal() {
		if constexpr (config::ENABLE_SYSTEM_LOAD || config::ENABLE_SYSTEM_LOAD_PER_THREAD) {

			if constexpr (config::ENABLE_SYSTEM_LOAD_PER_THREAD) {
#if AR_GLOBAL_OBJECT_LISTS
				std::array<List *const, 1> threadLists = {&g_ar_objects.threads};
#else
				std::array<List *const, 3> threadLists = {&Kernel::readyList(), &Kernel::suspendedList(), &Kernel::sleepingList()};
#endif // AR_GLOBAL_OBJECT_LISTS
	   // iterate over all known threads and reset their load counters
				for (auto &list : threadLists) {
					auto start = list->m_head;
					if (start) {
						auto node = start;
						do {
							auto *thread = node->getObject<Thread>();
							thread->systemLoad.reset();
							node = node->m_next;
						} while (node != start);
					}
				}
			}
			systemLoad.reset();
		}
	}

	//! @brief There is no more room available in the deferred action queue.
	void DEFERRED_ACTION_QUEUE_OVERFLOW_DETECTED() { Port::halt(); }

	Status Kernel::DeferredActionQueue::post(DeferredAction action, void *object) {

		decltype(m_last)::value_type index;
		auto result = insert(1, index);
		if (result != Status::success) {
			assert(false);
			return result;
		}

		m_entries[index].action = action;
		m_entries[index].object = object;

		Kernel::enter_scheduler();

		return Status::success;
	}

	Status Kernel::DeferredActionQueue::post(DeferredAction action, void *object, void *arg) {
		decltype(m_last)::value_type index;
		auto result = insert(2, index);
		if (result != Status::success) {
			assert(false);
			return result;
		}

		m_entries[index].action = action;
		m_entries[index].object = object;

		++index; // TODOFIXME: implement more robust iterator
		if (index >= config::DEFERRED_ACTION_QUEUE_SIZE) { index %= config::DEFERRED_ACTION_QUEUE_SIZE; }
		m_entries[index].action = reinterpret_cast<DeferredAction>(actionExtraValue);
		m_entries[index].object = arg;

		Kernel::enter_scheduler();

		return Status::success;
	}

	Status Kernel::DeferredActionQueue::insert(std::size_t entryCount, std::size_t &newEntryIndex) {
		auto count = m_count.load();
		do {
			count = m_count;
			if (count + entryCount > config::DEFERRED_ACTION_QUEUE_SIZE) { return Status::queueFullError; }
		} while (!m_count.compare_exchange_weak(count, count + entryCount));

		auto last = m_last.load();
		do { last = m_last; } while (!m_last.compare_exchange_weak(last, (last + entryCount) % config::DEFERRED_ACTION_QUEUE_SIZE));

		newEntryIndex = last;
		return Status::success;
	}
	//TODO: return refference or assert, except startup, there is no scenario with unknown current thread
	Thread *Thread::getCurrent() { return Kernel::getCurrent(); }
} // namespace Ar

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
