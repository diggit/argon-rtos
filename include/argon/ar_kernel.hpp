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
 * @file
 * @brief Header for the Argon RTOS C API.
 * @ingroup ar
 */

#if !defined(_AR_KERNEL_H_)
	#define _AR_KERNEL_H_

	#include "ar_config.hpp"
	#include "ar_common.hpp"
	#include "argon/ar_thread.hpp"
	#include "ar_list.hpp"
	#include "ar_atomic.hpp"
	#include "ar_assert.hpp"
	#include <optional>
	#include <array>
	#include <atomic>

namespace Ar {

	//------------------------------------------------------------------------------
	// Constants
	//------------------------------------------------------------------------------

	//! @brief Current version of Argon (v1.3.1).
	static constexpr std::uint32_t AR_VERSION = 0x00010301;

	extern std::array<std::uint8_t, config::IDLE_THREAD_STACK_SIZE> s_idleThreadStack;

	//------------------------------------------------------------------------------
	// Types
	//------------------------------------------------------------------------------

	// Forward declarations.
	struct Thread;
	struct Channel;
	struct Queue;
	struct Timer;
	struct Runloop;

	//! @brief Callback routine for timer expiration.
	//!

	/*!
	 * @brief Argon kernel state.
	 */
	class Kernel {

		//! @brief Queue containing deferred actions.
		//!
		//! The deferred action queue is used to postpone kernel operations performed in interrupt context
		//! until the scheduler runs, in the lowest possible interrupt priority. This is part of the
		//! support for never disabling interrupts on Cortex-M.
		//!
		//! The deferred actions enqueued in this queue are composed of a function pointer and object
		//! value. The function pointer points to a very small deferred action stub function that simply
		//! calls the right kernel object operation routine with the correct parameters. If a deferred
		//! action requires two parameters, a special #actionExtraValue constant can be inserted
		//! in the queue to mark an entry as holding the second parameter for the previous action.
		struct DeferredActionQueue {
			//! @brief Constant used to mark an action queue entry as containing an extra argument for the
			//!     previous action.
			static constexpr std::uint32_t actionExtraValue = 0xfeedf00d;

			//! @brief The deferred action function pointer.
			using DeferredAction = void (*)(void *object, void *object2);

			std::atomic<std::size_t> m_count {0}; //!< Number of queue entries.
			std::atomic<std::size_t> m_first {0}; //!< First entry index.
			std::atomic<std::size_t> m_last {0}; //!< Last entry index.
			struct DeferredActionQueueEntry {
				DeferredAction action {nullptr}; //!< Enqueued action.
				void *object {nullptr}; //!< Kernel object or parameter for enqueued action.
			};
			std::array<DeferredActionQueueEntry, config::DEFERRED_ACTION_QUEUE_SIZE> m_entries {}; //!< The deferred action queue entries.

			//! @brief Returns whether the queue is currently empty.
			bool isEmpty() const { return m_count == 0; }
			bool isEmpty() { return m_count == 0; }

			//! @brief Enqueues a new deferred action.
			Status post(DeferredAction action, void *object);

			//! @brief Enqueues a new deferred action with two arguments.
			Status post(DeferredAction action, void *object, void *arg);

			constexpr DeferredActionQueue() {};

		  protected:
			//! @brief Reserves room to insert a number of new entries.
			//! @return First index in queue where the requested number of entries can be inserted. If there
			//!     is not room in the queue for the requested entries, then -1 is returned.
			Status insert(std::size_t entryCount, std::size_t &newEntryIndex);
		};

		Thread *m_currentThread {nullptr}; //!< The currently running thread.
		List m_readyList {Thread::sort_by_priority}; //!< List of threads ready to run.
		List m_suspendedList {}; //!< List of suspended threads.
		List m_sleepingList {Thread::sort_by_wakeup}; //!< List of sleeping threads.
		struct KernelFlags {
			bool isRunning : 1 {false}; //!< True if the kernel has been started.
			bool needsReschedule : 1 {false}; //!< True if we need to reschedule once the kernel is unlocked.
			bool isRunningDeferred : 1 {false}; //!< True if the kernel is executing deferred actions.
			bool needsRoundRobin : 1 {false}; //!< True if round-robin scheduling must be used.
			std::uint32_t _reservedFlags : 28 {};
		} m_flags {}; //!< Kernel flags.
		DeferredActionQueue m_deferredActions {}; //!< Actions deferred from interrupt context.
		std::atomic<unsigned int> m_lockCount {}; //!< Whether the kernel is locked.
		std::optional<Time> m_nextWakeup {}; //!< Time of the next wakeup event.
		unsigned int m_threadIdCounter ; //!< Counter for generating unique thread IDs.

		class SystemLoadDisabled {
			constexpr void reset() {}
			constexpr Time getLastSwitchIn() { return Time::zero(); }
			constexpr Time getLoadtStart() { return Time::zero(); }
			constexpr void recordSwitch() {}
			constexpr void accumulate(Duration duration) {};
			constexpr Duration getAccumulated(void) { return Duration::zero(); }
		};

		class SystemLoadEnabled {
		  public:
			Time lastLoadStart {Time::now()}; //!< timestamp for last load computation start.
			Time lastSwitchIn {Time::now()}; //!< timestamp when current thread was switched in.
			Duration busyTime {Duration::zero()}; //!< accumulated time spent in threads other than idle thread
			std::uint16_t systemLoad {}; //!< Per mille of system load from 0-1000.

		  public:
			void reset() {
				lastLoadStart = Time::now();
				lastSwitchIn = Time::now();
				busyTime = Duration::zero();
			}
			Time getLastSwitchIn() { return lastSwitchIn; }
			Time getLoadtStart() { return lastLoadStart; }
			void recordSwitch() { lastSwitchIn = Time::now(); }
			void accumulate(Duration duration) { busyTime += duration; }
			Duration getAccumulated(void) { return busyTime; }
		};

		class SystemLoad :
			public std::conditional_t<config::ENABLE_SYSTEM_LOAD || config::ENABLE_SYSTEM_LOAD_PER_THREAD, SystemLoadEnabled, SystemLoadDisabled> {
			using super = std::conditional_t<config::ENABLE_SYSTEM_LOAD || config::ENABLE_SYSTEM_LOAD_PER_THREAD, SystemLoadEnabled, SystemLoadDisabled>;
			using super::super;
		};

		[[no_unique_address]] SystemLoad systemLoad {};

		Thread m_idleThread; //!< The lowest priority thread in the system. Executes only when no other threads are ready.

	  public:
		Kernel() :
			m_idleThread("IDLE", idle_entry, nullptr, s_idleThreadStack.data(), std::size(s_idleThreadStack), Thread::idleThreadPriority,
				Thread::InitialState::suspended) {}

		Kernel(const Kernel &) = delete;
		Kernel(const Kernel &&) = delete;
		Kernel &operator=(const Kernel &) = delete;

		void run_internal();
		static inline void run();

		unsigned int getNewThreadId_internal() { return ++m_threadIdCounter; }
		static inline unsigned int getNewThreadId();

		static inline Thread *getCurrent();
		static inline Status postDeferredAction(DeferredActionQueue::DeferredAction action, void *object);
		static inline Status postDeferredAction(DeferredActionQueue::DeferredAction action, void *object, void *object2);

		std::uint32_t yieldIsr_internal(std::uint32_t topOfStack);
		static inline std::uint32_t yieldIsr(std::uint32_t topOfStack); // called from Port

		void timerIsr_internal();
		static inline void timerIsr();

		void enter_scheduler_internal();
		static inline void enter_scheduler();

		void update_round_robin_internal();
		static inline void update_round_robin();

		List &readyList_internal() { return m_readyList; }
		static inline List &readyList();

		List &suspendedList_internal() { return m_suspendedList; }
		static inline List &suspendedList();

		List &sleepingList_internal() { return m_sleepingList; }
		static inline List &sleepingList();

		Thread *idleThread_internal() { return &m_idleThread; }
		static inline Thread *idleThread();

		std::uint16_t get_system_load_internal(bool reset);
		static inline std::uint16_t get_system_load(bool reset);

		std::uint16_t get_thread_load_internal(Thread *thread);
		static inline std::uint16_t get_thread_load(Thread *thread);

		void reset_system_load_internal();
		static inline void reset_system_load();

		static inline decltype(m_lockCount) &lockCount();

		static inline decltype(m_flags) &flags();

		static inline bool is_running(void);

		static inline void halt() { Port::halt(); } // this just kills system

		static void idle_entry(void *param); // public because Thread checks for valid priority and must do exception for idle thread
	  private:
		bool process_ticks(bool fromTimerIrq);
		void run_deferred_actions();
		void scheduler();
		std::optional<Time> get_next_wakeup_time();
		void run_timers(List &timersList);
		std::int32_t atomic_queue_insert(std::int32_t entryCount, volatile std::int32_t &qCount, volatile std::int32_t &qTail, std::int32_t qSize);
		void runloop_wake(Runloop *runloop);

		// Thread *getCurrent() { return currentThread; }

		//! Static members to get system-wide information.
		//@{
		//! @brief Returns the currently running thread object.
		//@}
	};

	//! Contains linked lists of all the various Ar object types that have been created during runtime.

	struct AllObjectsDisabled {};
	struct AllObjectsEnabled {
		List threads; //!< All existing threads.
		List semaphores; //!< All existing semaphores.
		List mutexes; //!< All existing mutexes.
		List channels; //!< All existing channels;
		List queues; //!< All existing queues.
		List timers; //!< All existing timers.
		List runloops; //!< All existing runloops.
	};

	struct AllObjects : public std::conditional_t<config::GLOBAL_OBJECT_LISTS, AllObjectsEnabled, AllObjectsDisabled> {
		using super = std::conditional_t<config::GLOBAL_OBJECT_LISTS, AllObjectsEnabled, AllObjectsDisabled>;
		using super::super;
	};

	extern AllObjects allObjects;

	extern Kernel g_ar;

	// static wrappers for kernel
	inline Status Kernel::postDeferredAction(Kernel::DeferredActionQueue::DeferredAction action, void *object) {
		return g_ar.m_deferredActions.post(action, object);
	}
	inline Status Kernel::postDeferredAction(Kernel::DeferredActionQueue::DeferredAction action, void *object, void *object2) {
		return g_ar.m_deferredActions.post(action, object, object2);
	}
	inline std::uint32_t Kernel::yieldIsr(std::uint32_t topOfStack) { return g_ar.yieldIsr_internal(topOfStack); }
	inline void Kernel::timerIsr() { g_ar.timerIsr_internal(); }
	inline void Kernel::update_round_robin() { g_ar.update_round_robin_internal(); }
	inline List &Kernel::readyList() { return g_ar.readyList_internal(); }
	inline List &Kernel::suspendedList() { return g_ar.suspendedList_internal(); }
	inline List &Kernel::sleepingList() { return g_ar.sleepingList_internal(); }
	inline Thread *Kernel::idleThread() { return g_ar.idleThread_internal(); }
	inline void Kernel::enter_scheduler() { g_ar.enter_scheduler_internal(); }
	inline decltype(Kernel::m_lockCount) &Kernel::lockCount() { return g_ar.m_lockCount; }
	inline decltype(Kernel::m_flags) &Kernel::flags() { return g_ar.m_flags; }
	inline unsigned int Kernel::getNewThreadId() { return g_ar.getNewThreadId_internal(); }
	inline bool Kernel::is_running(void) { return g_ar.m_flags.isRunning; }
	inline void Kernel::run() { g_ar.run_internal(); }
	inline Thread *Kernel::getCurrent() { return g_ar.m_currentThread; }
	inline std::uint16_t Kernel::get_system_load(bool reset) { return g_ar.get_system_load_internal(reset); }
	inline std::uint16_t Kernel::get_thread_load(Thread *thread) { return g_ar.get_thread_load_internal(thread); }
	inline void Kernel::reset_system_load() { g_ar.reset_system_load_internal(); }

	/*!
	 * @brief Utility class to temporarily lock or unlock the kernel.
	 *
	 * @param E The desired lock state, true for locked and false for unlocked.
	 */
	template<bool E>
	class KernelGuard {
	  public:
		//! @brief Saves lock state then modifies it.
		KernelGuard() {
			if (E) {
				Kernel::lockCount()++;
			} else {
				Ar::assert(Kernel::lockCount() != 0);
				Kernel::lockCount()--;
			}
		}

		//! @brief Restores previous lock state.
		~KernelGuard() {
			if (E) {
				assert(Kernel::lockCount() != 0);
				Kernel::lockCount()--;
				if (Kernel::lockCount() == 0 && Kernel::flags().needsReschedule && !Kernel::flags().isRunningDeferred) { Kernel::enter_scheduler(); }
			} else {
				Kernel::lockCount()++;
			}
		}
	};

	using KernelLock = KernelGuard<true>; //!< Lock kernel.
	using KernelUnlock = KernelGuard<false>; //!< Unlock kernel.s

} // namespace Ar

#endif // _AR_KERNEL_H_