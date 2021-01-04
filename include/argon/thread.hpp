#if !defined(_AR_THREAD_HDR_INCLUDED)
	#define _AR_THREAD_HDR_INCLUDED

	#include "argon/common.hpp"
	#include "argon/list.hpp"
	#include "argon/port.hpp"
	#include "argon/units.hpp"
	#include "argon/config.hpp"
	#include "argon/runloop.hpp"
	#include "argon/assert.hpp"
	#include <cstring>
	#include <cstdint>
	#include <optional>

//! @brief The Argon RTOS namespace.
namespace Ar {

	// class Runloop;

	//------------------------------------------------------------------------------
	// Classes
	//------------------------------------------------------------------------------

	/*!
	 * @brief Preemptive thread class.
	 *
	 * @ingroup ar_thread
	 *
	 * This thread class implements a preemptive thread with variable priority.
	 *
	 * Threads may be allocated either globally or with the new operator. You can also allocate a
	 * thread locally, but you must be careful to keep the stack valid as long as the thread is running.
	 * There are two options for initialization. Either use one of the non-default constructors, or
	 * use the default constructor and call an init() method at some later point. Both the constructors
	 * and init methods take the same arguments. They accept a name, entry point, stack, and priority.
	 *
	 * The entry point can be a global or static function that matches the #Entry
	 * prototype. Alternatively, there are constructor and init() variants that let you use a member
	 * function of a specific object as the entry point.
	 *
	 * The init() method leaves the new thread suspended. To make the new thread eligible to
	 * run you must call the resume() method on it.
	 */
	class Thread {
	  public:
		using Priority = std::uint8_t;

		//! @brief Prototype for the thread entry point.
		//!
		//! @ingroup ar_thread
		using Entry = void (*)(void *param);

		//! @brief Options for creating a new thread.
		//!
		//! These constants are meant to be used for the @a initialState parameter.
		enum class InitialState : bool {
			running = true, //!< Automatically run the thread.
			suspended = false //!< Create the thread suspended.
		};

		//! @brief Potential thread states.
		//!
		//! @ingroup ar_thread
		enum class ThreadState {
			unknown, //!< Hopefully a thread is never in this state.
			suspended, //!< Thread is not eligible for execution.
			ready, //!< Thread is eligible to be run.
			running, //!< The thread is currently running.
			blocked, //!< The thread is blocked on another object.
			sleeping, //!< Thread is sleeping.
			done //!< Thread has exited.
		};

		//! @brief Range of priorities for threads.
		//!
		//! @ingroup ar_thread
		static constexpr Priority idleThreadPriority = 0; //!< The idle thread's priority. No other thread is allowed to have this priority.
		static constexpr Priority minThreadPriority = 1; //!< Priority value for the lowest priority user thread.
		static constexpr Priority maxThreadPriority = 255; //!< Priority value for the highest priority user thread.

		volatile std::uint8_t *m_stackPointer {nullptr}; //!< Current stack pointer.
		Port::ThreadData m_portData {}; //!< Port-specific thread data.
		Name m_name {}; //!< Thread name.
		std::uint32_t *m_stackBottom {nullptr}; //!< Beginning of stack.
		Priority m_priority; //!< Thread priority. 0 is the lowest priority.
		ThreadState m_state {ThreadState::suspended}; //!< Current thread state.
		Entry m_entry {nullptr}; //!< Function pointer for the thread's entry point.
		List::Node m_threadNode; //!< Main thread list node.

	#if AR_GLOBAL_OBJECT_LISTS
		List::Node m_createdNode; //!< Created list node.
	#endif // AR_GLOBAL_OBJECT_LISTS

		List::Node m_blockedNode; //!< Blocked list node.
		std::optional<Time> m_wakeupTime {}; //!< Tick count when a sleeping thread will awaken.
		StatusW m_unblockStatus {Status::success}; //!< Status code to return from a blocking function upon unblocking.
		void *m_channelData {nullptr}; //!< Receive or send data pointer for blocked channel.
		Runloop *m_runLoop {nullptr}; //!< Run loop associated with this thread.

		struct SystemLoadDisabled {
			constexpr Duration getAccumulator() { return Duration::zero(); }
			constexpr void accumulate(Duration duration) {}
			constexpr void reset() {}
		};

		struct SystemLoadEnabled {
			// FIXME: shouldn't this be atomic?
			Duration m_loadAccumulator {Duration::zero()}; //!< Duration this thread has run during the current load computation period.

			Duration getAccumulator() { return m_loadAccumulator; }
			void accumulate(Duration duration) { m_loadAccumulator += duration; }
			void reset() { m_loadAccumulator = Duration::zero(); }
		};

		using SystemLoad = std::conditional_t<config::ENABLE_SYSTEM_LOAD_PER_THREAD, SystemLoadEnabled, SystemLoadDisabled>;
		[[no_unique_address]] SystemLoad systemLoad {};

		std::uint32_t *m_stackTop {nullptr}; //!< Saved stack top address for computing stack usage.
		std::uint32_t m_uniqueId {}; //!< Unique ID for this thread. 

		// Internal utility methods.
		void block(List &blockedList, std::optional<Duration> timeout);
		void unblockWithStatus(List &blockedList, Status unblockStatus);

		/*!
		 * @brief Current status of a thread.
		 *
		 * This struct holds a report on the current status of a thread. The ar_thread_get_report()
		 * API will fill in an array of these structs with the status of all threads.
		 *
		 * @ingroup ar_thread
		 */
		struct ThreadStatus {
			Thread *m_thread; //!< Pointer to the thread's structure.
			Name m_name {}; //!< Thread's name.
			std::uint32_t m_uniqueId {}; //!< Unique ID for this thread.
			std::uint32_t m_cpu; //!< Per mille CPU usage of the thread over the last sampling period, with a range of 1-1000.
			ThreadState m_state; //!< Current thread state.
			std::size_t m_maxStackUsed; //!< Maximum number of bytes used in the thread's stack.
			std::size_t m_stackSize; //!< Total bytes allocated to the thread's stack.

			constexpr ThreadStatus(
				Thread *thread, Name &name, std::uint32_t uniqueId, std::uint32_t cpu, ThreadState state, std::size_t maxStackUsed, std::size_t stackSize) :
				m_thread(thread),
				m_name(name),
				m_uniqueId(uniqueId),
				m_cpu(cpu),
				m_state(state),
				m_maxStackUsed(maxStackUsed),
				m_stackSize(stackSize) {};

			constexpr ThreadStatus(Thread *thread, const char *name, std::uint32_t uniqueId, std::uint32_t cpu, ThreadState state, std::size_t maxStackUsed,
				std::size_t stackSize) :
				m_thread(thread),
				m_name(),
				m_uniqueId(uniqueId),
				m_cpu(cpu),
				m_state(state),
				m_maxStackUsed(maxStackUsed),
				m_stackSize(stackSize) {
				std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);
			};
		};

		std::size_t getReport(ThreadStatus *report, std::size_t maxEntries, Duration loadMeasurementTime);

		//! @brief Default constructor.
		Thread() = delete;
		//! @brief Constructor.
		//!
		//! @param name Name of the thread. If NULL, the thread's name is set to an empty string.
		//! @param entry Thread entry point taking one parameter and returning void.
		//! @param param Arbitrary pointer-sized value passed as the single parameter to the thread
		//!     entry point.
		//! @param stack Pointer to the start of the thread's stack. This should be the stack's bottom,
		//!     not it's top. If this parameter is NULL, the stack will be dynamically allocated.
		//! @param stackSize Number of bytes of stack space allocated to the thread. This value is
		//!     added to @a stack to get the initial top of stack address.
		//! @param priority Thread priority. The accepted range is 1 through 255. Priority 0 is
		//!     reserved for the idle thread.
		//! @param initialState Whether the new thread will start to run automatically. If false, the
		//!     thread will be created in a suspended state. The constants #kArStartThread and
		//!     #kArSuspendThread can be used to better document this parameter value.
		Thread(const char *name, Entry entry, void *param, void *stack, std::size_t stackSize, Priority priority,
			InitialState initialState = InitialState::running);

		//! @brief Constructor to set the thread entry to a member function.
		//!
		//! @param name Name of the thread. If NULL, the thread's name is set to an empty string.
		//! @param object Pointer to an instance of class T upon which the @a entry member function
		//!     will be invoked when the thread is started.
		//! @param entry Member function of class T that will be used as the thread's entry point.
		//!     The member function must take no parameters and return void.
		//! @param stack Pointer to the start of the thread's stack. This should be the stack's bottom,
		//!     not it's top. If this parameter is NULL, the stack will be dynamically allocated.
		//! @param stackSize Number of bytes of stack space allocated to the thread. This value is
		//!     added to @a stack to get the initial top of stack address.
		//! @param priority Thread priority. The accepted range is 1 through 255. Priority 0 is
		//!     reserved for the idle thread.
		//! @param initialState Whether the new thread will start to run automatically. If false, the
		//!     thread will be created in a suspended state. The constants #kArStartThread and
		//!     #kArSuspendThread can be used to better document this parameter value.
		template<class T>
		Thread(const char *name, void (T::*entry)(), T *object, void *stack, std::size_t stackSize, Priority priority,
			InitialState initialState = InitialState::running) :
			Thread(name, member_thread_entry<T>, object, stack, stackSize, priority, initialState) {

			// Save the member function pointer on the thread's stack. Add 1 to skip over check value.
			std::uint32_t *storage = m_stackBottom + 1;
			std::memcpy(storage, &entry, sizeof(entry));
		}

		//! @brief Constructor to dynamically allocate the stack.
		//!
		//! @param name Name of the thread. If NULL, the thread's name is set to an empty string.
		//! @param entry Thread entry point taking one parameter and returning void.
		//! @param param Arbitrary pointer-sized value passed as the single parameter to the thread
		//!     entry point.
		//! @param stackSize Number of bytes of stack space to allocate via <tt>new</tt> to the thread.
		//! @param priority Thread priority. The accepted range is 1 through 255. Priority 0 is
		//!     reserved for the idle thread.
		//! @param initialState Whether the new thread will start to run automatically. If false, the
		//!     thread will be created in a suspended state. The constants #kArStartThread and
		//!     #kArSuspendThread can be used to better document this parameter value.
		Thread(const char *name, Entry entry, void *param, std::size_t stackSize, Priority priority, InitialState initialState = InitialState::running) :
			Thread(name, entry, param, nullptr, stackSize, priority, initialState) {}

		//! @brief Constructor to set the thread entry to a member function, using a dynamic stack.
		//!
		//! @param name Name of the thread. If NULL, the thread's name is set to an empty string.
		//! @param object Pointer to an instance of class T upon which the @a entry member function
		//!     will be invoked when the thread is started.
		//! @param entry Member function of class T that will be used as the thread's entry point.
		//!     The member function must take no parameters and return void.
		//! @param stackSize Number of bytes of stack space to allocate via <tt>new</tt> to the thread.
		//! @param priority Thread priority. The accepted range is 1 through 255. Priority 0 is
		//!     reserved for the idle thread.
		//! @param initialState Whether the new thread will start to run automatically. If false, the
		//!     thread will be created in a suspended state. The constants #kArStartThread and
		//!     #kArSuspendThread can be used to better document this parameter value.
		template<class T>
		Thread(const char *name, void (T::*entry)(), T *object, std::size_t stackSize, Priority priority, InitialState initialState = InitialState::running) :
			Thread(name, member_thread_entry<T>, object, stackSize, priority, initialState) {

			// Save the member function pointer on the thread's stack. Add 1 to skip over check value.
			std::uint32_t *storage = m_stackBottom + 1;
			std::memcpy(storage, &entry, sizeof(entry));
		}

		//! @brief Destructor.
		virtual ~Thread();

		//! @name Thread init and cleanup
		//@{
		//! @brief Base initialiser.
		//!
		//! The thread is in suspended state when this method exits. To make it eligible for
		//! execution, call the resume() method.
		//!
		//! @param name Name of the thread. If NULL, the thread's name is set to an empty string.
		//! @param entry Thread entry point taking one parameter and returning void.
		//! @param param Arbitrary pointer-sized value passed as the single parameter to the thread
		//!     entry point.
		//! @param stack Pointer to the start of the thread's stack. This should be the stack's bottom,
		//!     not it's top. If this parameter is NULL, the stack will be dynamically allocated.
		//! @param stackSize Number of bytes of stack space allocated to the thread. This value is
		//!     added to @a stack to get the initial top of stack address.
		//! @param priority Thread priority. The accepted range is 1 through 255. Priority 0 is
		//!     reserved for the idle thread.
		//! @param initialState Whether the new thread will start to run automatically. If false, the
		//!     thread will be created in a suspended state. The constants #kArStartThread and
		//!     #kArSuspendThread can be used to better document this parameter value.
		//!
		//! @retval #Status::success The thread was initialised without error.
		//! @retval #kArOutOfMemoryError Failed to dynamically allocate the stack.
		StatusW init(const char *name, Entry entry, void *param, void *stack, std::size_t stackSize, Priority priority,
			InitialState initialState = InitialState::running);

		//! @brief Initializer to set the thread entry to a member function.
		//!
		//! @param name Name of the thread. If NULL, the thread's name is set to an empty string.
		//! @param object Pointer to an instance of class T upon which the @a entry member function
		//!     will be invoked when the thread is started.
		//! @param entry Member function of class T that will be used as the thread's entry point.
		//!     The member function must take no parameters and return void.
		//! @param stack Pointer to the start of the thread's stack. This should be the stack's bottom,
		//!     not it's top. If this parameter is NULL, the stack will be dynamically allocated.
		//! @param stackSize Number of bytes of stack space allocated to the thread. This value is
		//!     added to @a stack to get the initial top of stack address.
		//! @param priority Thread priority. The accepted range is 1 through 255. Priority 0 is
		//!     reserved for the idle thread.
		//! @param initialState Whether the new thread will start to run automatically. If false, the
		//!     thread will be created in a suspended state. The constants #kArStartThread and
		//!     #kArSuspendThread can be used to better document this parameter value.
		//!
		//! @retval #Status::success The thread was initialised without error.
		//! @retval #kArOutOfMemoryError Failed to dynamically allocate the stack.
		template<class T>
		StatusW init(const char *name, T *object, void (T::*entry)(), void *stack, std::size_t stackSize, Priority priority,
			InitialState initialState = InitialState::running) {
			return initForMemberFunction(name, object, member_thread_entry<T>, &entry, sizeof(entry), stack, stackSize, priority, initialState);
		}
		//@}

		//! @brief Get the thread's name.
		const char *getName() const { return m_name.data(); }

		//! @name Thread state
		//!
		//! Control of and access to the thread state.
		//@{
		//! @brief Put thread in suspended state.
		//!
		//! If this method is called from the current thread then the scheduler is entered immediately
		//! after putting the thread on the suspended list. Calling suspend() on another thread will not
		//! cause the scheduler to switch threads.
		//!
		//! Does not enter the scheduler if Ar is not running. Does nothing if the thread is already
		//! suspended.
		//!
		//! @todo Deal with all thread states properly.
		StatusW suspend();

		//! @brief Make the thread eligible for execution.
		//!
		//! If the thread being resumed has a higher priority than that of the current thread, the
		//! scheduler is called to immediately switch threads. In this case the thread being resumed
		//! will always become the new current thread. This is because the highest priority thread is
		//! always guaranteed to be running, meaning the calling thread was the previous highest
		//! priority thread.
		//!
		//! Does not enter the scheduler if Ar is not running. Does nothing if the thread is already on
		//! the ready list.
		//!
		//! @todo Deal with all thread states properly.
		StatusW resume();

		static void deferred_resume(void *object, void *object2);

		static void deferred_suspend(void *object, void *object2);

		//! @brief Return the current state of the thread.
		ThreadState getState() const { return m_state; }

		//! @brief Put the current thread to sleep for a certain amount of time.
		//!
		//! Does nothing if Ar is not running.
		//!
		//! A sleeping thread can be woken early by calling ar_thread_resume().
		//!
		//! @param milliseconds The number of milliseconds to sleep the calling thread. A sleep time
		//!     of 0 is ignored. If the sleep time is shorter than the scheduler quanta, then the thread
		//!     will not actually sleep. If #kArInfiniteTimeout is passed for the sleep time, the thread
		//!     will simply be suspended.
		static void sleep(std::optional<Duration> duration = std::nullopt);

		//! @brief Put the current thread to sleep until a specific time.
		//!
		//! Does nothing if Ar is not running.
		//!
		//! A sleeping thread can be woken early by calling ar_thread_resume().
		//!
		//! @param wakeup The wakeup time in milliseconds. If the time is not in the future, i.e., less than
		//!  or equal to the current value returned by ar_get_millisecond_count(), then the sleep request is
		//!  ignored.
		static void sleepUntil(Time time);
		//@}

		//! @name Thread priority
		//!
		//! Accessors for the thread's priority.
		//@{
		//! @brief Return the thread's current priority.
		Priority getPriority() const { return m_priority; }

		//! @brief Change the thread's priority.
		//!
		//! The scheduler is invoked after the priority is set so that the current thread can be changed
		//! to the one with the highest priority. The scheduler is invoked even if there is no new
		//! highest priority thread. In this case, control may switch to the next thread with the same
		//! priority, assuming there is one.
		//!
		//! Does not enter the scheduler if Ar is not running.
		//!
		//! @param priority Thread priority level from 1 to 255, where lower numbers have a lower
		//!     priority. Priority number 0 is not allowed because it is reserved for the idle thread.
		//!
		//! @retval #Status::success
		//! @retval #kArInvalidPriorityError
		StatusW setPriority(Priority priority);
		//@}

		//! @name Info
		//@{
		//! @brief Get the thread's system load.
		std::uint16_t get_load(Duration measurementTime);

		//! @brief Get the thread's maximum stack usage.
		std::size_t getStackUsedMax();

		std::size_t getStackUsedNow();
		//@}

		//! @retval true The @a a thread has a higher priority than @a b.
		//! @retval false The @a a thread has a lower or equal priority than @a b.
		static bool sort_by_priority(List::Node *a, List::Node *b) {
			Thread *aThread = a->getObject<Thread>();
			Thread *bThread = b->getObject<Thread>();
			return (aThread->m_priority > bThread->m_priority);
		}

		//! @retval true The @a a thread has an earlier wakeup time than @a b.
		//! @retval false The @a a thread has a later or equal wakeup time than @a b.
		static bool sort_by_wakeup(List::Node *a, List::Node *b) {
			Thread *aThread = a->getObject<Thread>();
			Thread *bThread = b->getObject<Thread>();
			return (aThread->m_wakeupTime < bThread->m_wakeupTime);
		}

		//! @name Thread entry point wrapper
		//@{
		//! @brief Thread entry point.
		static void wrapper(Thread *thread, void *param);
		//@}

		static Thread *getCurrent();

	  protected:
		std::uint8_t *m_allocatedStack {nullptr}; //!< Dynamically allocated stack.
		Entry m_userEntry {nullptr}; //!< User-specified thread entry point function.

		//! @brief Virtual thread entry point.
		//!
		//! This is the method that subclasses should override.
		virtual void threadEntry(void *param);

		//! @brief Static thread entry callback to invoke the virtual method.
		static void thread_entry(void *param);

		//! @brief Template function to invoke a thread entry point that is a member function.
		template<class T>
		static void member_thread_entry(void *param) {
			Thread *thread = Thread::getCurrent();
			T *obj = static_cast<T *>(param);
			void (T::*member)(void);
			std::uint32_t *storage = thread->m_stackBottom + 1; // Add 1 to skip over check value.
			std::memcpy(&member, storage, sizeof(member));
			(obj->*member)();
		}

		//! @brief Special init method to deal with member functions.
		StatusW initForMemberFunction(const char *name, void *object, Entry entry, void *memberPointer, std::size_t memberPointerSize, void *stack,
			std::size_t stackSize, Priority priority, InitialState initialState);

	  private:
		//! @brief The copy constructor is disabled for thread objects.
		Thread(const Thread &other);

		//! @brief Disable assignment operator.
		Thread &operator=(const Thread &other);

		StatusW resume_internal();

		StatusW suspend_internal();
	}; // namespace Ar

	/*!
	 * @brief Template to create a thread and its stack.
	 *
	 * @ingroup ar_thread
	 */
	template<std::size_t S>
	class ThreadWithStack : public Thread {
	  public:
		//! @brief Default constructor.
		ThreadWithStack() {}

		//! @brief Constructor to use a normal function as entry poin.
		ThreadWithStack(const char *name, Entry entry, void *param, Priority priority, InitialState initialState = InitialState::running) :
			Thread(name, entry, param, m_stack, S, priority, initialState) {};

		//! @brief Constructor to set the thread entry to a member function.
		template<class T>
		ThreadWithStack(const char *name, void (T::*entry)(), T *object, Priority priority, InitialState initialState = InitialState::running) :
			Thread(name, entry, object, m_stack, S, priority, initialState) {}

	  protected:
		uint8_t m_stack[S]; //!< Stack space for the thread.

	  private:
		//! @brief The copy constructor is disabled for thread objects.
		ThreadWithStack(const ThreadWithStack<S> &other);

		//! @brief Disable assignment operator.
		ThreadWithStack &operator=(const ThreadWithStack<S> &other);
	};

	class TemporaryPriority {
		Thread *const thread;
		const Ar::Thread::Priority originalPriority;

	  public:
		TemporaryPriority(Thread *const thread_, const Ar::Thread::Priority priority_) : thread(thread_), originalPriority(thread->getPriority()) {
			assert(thread_);
			assertEval(thread->setPriority(priority_));
		}

		// temporarily (RAII) changes current thread priority
		TemporaryPriority(const Ar::Thread::Priority priority_) : thread(Thread::getCurrent()), originalPriority(thread->getPriority()) {
			assertEval(thread->setPriority(priority_));
		}

		~TemporaryPriority() { assertEval(thread->setPriority(originalPriority)); }

		TemporaryPriority(const Ar::TemporaryPriority &) = delete;
		TemporaryPriority(const Ar::TemporaryPriority &&) = delete;

		TemporaryPriority &operator=(const Ar::TemporaryPriority &) = delete;

		template<typename... Args>
		void *operator new(std::size_t, Args...) = delete;
	};
} // namespace Ar
#endif