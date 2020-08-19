#if !defined(_AR_RUNLOOP_H_)
	#define _AR_RUNLOOP_H_

	#include "ar_list.hpp"
	#include "ar_config.hpp"
	#include "ar_common.hpp"
	#include <optional>
	#include <array>
	#include <atomic>

namespace Ar {
	/*!
	 * @brief Run loop.
	 *
	 * @ingroup ar_runloop
	 */
	struct Thread;
	struct Queue;
	struct Channel;
	struct Timer;

	class Runloop {
		friend Timer;
		friend Thread;

	  public:
		/*!
		 * @brief Run loop result.
		 *
		 * @ingroup ar_runloop
		 */
		union RunloopResult {
			Queue *m_queue; //!< Queue that received an item.
		};

		//! @brief
		using RunloopFunction = void (*)(void *param);

		//! @brief
		using RunloopQueueHandler = void (*)(Queue *queue, void *param);

		//! @brief
		using RunloopChannelHandler = void (*)(Channel *channel, void *param);

	  protected:
		/*!
		 * @brief Run loop.
		 *
		 * @ingroup ar_runloop
		 */
		Name m_name; //!< Name of the runloop.
		Thread *m_thread {nullptr}; //!< Thread the runloop is running on. nullptr when the runloop is not running.
		List m_timers; //!< Timers associated with the runloop.
		List m_queues {}; //!< Queues associated with the runloop.
		struct RunloopFunctionInfo {
			RunloopFunction function {nullptr}; //!< The callback function pointer.
			void *param {nullptr}; //!< User parameter passed to the callback.
		};
		std::array<RunloopFunctionInfo, config::RUNLOOP_FUNCTION_QUEUE_SIZE> m_functions {}; //!< Function queue.
		std::atomic<std::size_t> m_functionCount {}; //!< Number of functions in the queue.
		std::atomic<std::size_t> m_functionHead {}; //!< Function queue head.
		std::atomic<std::size_t> m_functionTail {}; //!< Function queue tail.
		bool m_isRunning {false}; //!< Whether the runloop is currently running.
		volatile bool m_stop {false}; //!< Flag to force the runloop to stop.
	#if AR_GLOBAL_OBJECT_LISTS
		List::Node m_createdNode; //!< Created list node.
	#endif // AR_GLOBAL_OBJECT_LISTS

	  public:
		//! @brief Default constructor.
		// just no copying moving or so...
		Runloop() = delete;
		Runloop(const Runloop &) = delete;
		Runloop(const Runloop &&) = delete;
		Runloop &operator=(const Ar::Runloop &) = delete;

		//! @brief Constructor to init the runloop.
		//! @param The runloop's name. May be nullptr.
		Runloop(const char *name);

		//! @brief Runloop destructor.
		~Runloop();

		//! @brief Get the run loop's name.
		const char *getName() const { return m_name.data(); }

		/*!
		 * @brief Run a runloop for a period of time.
		 *
		 * Starts the runloop running for a specified amount of time. It will sleep the thread until an
		 * associated timer or source is pending. If the timeout expires, the API will return. To force the
		 * runloop to exit, call ar_runloop_stop().
		 *
		 * It's ok to nest runs of the runloop, but only on a single thread.
		 *
		 * If a queue is associated with the runloop (via ar_runloop_add_queue()) and the queue receives
		 * an item, then the runloop will either invoke the queue handler callback or exit. If it exits,
		 * it will return #kArRunloopQueueReceived and the @a object parameter will be filled in with the
		 * queue object that received the item. If @a object is nullptr, then the runloop will still exit but
		 * you cannot tell which queue received. This is acceptable if only one queue is associated with
		 * the runloop.
		 *
		 * @param timeout The maximum number of milliseconds to run the runloop. If this value is 0, or
		 *      #kArNoTimeout, then the call will exit after handling pending sources. Setting the timeout to
		 *      #kArInfiniteTimeout will cause the runloop to run until stopped or a queue receives an item.
		 * @param[out] object Optional structure that will be filled in when the return value is #kArRunloopQueueReceived.
		 *      May be nullptr, in which case the receiving queue cannot be indicated.
		 *
		 * @retval #kArRunloopStopped The runloop exited due to a timeout or explict call to ar_runloop_stop().
		 * @retval #kArRunloopQueueReceived A queue associated with the runloop received an item.
		 * @retval #kArTimeoutError The runloop timed out.
		 * @retval #kArRunloopAlreadyRunningError The runloop is already running on another thread, or another
		 *      runloop is already running on the current thread.
		 * @retval #Ar::Status::notFromInterruptError Cannot run a runloop from interrupt context.
		 */
		Ar::Status run(RunloopResult *object = nullptr, std::optional<Duration> timeout = std::nullopt);

		/*!
		 * @brief Stop a runloop.
		 *
		 * Use this function to stop a running runloop. It may be called from any execution context, including
		 * from within the runloop itself, another thread, or interrupt context. When the runloop stops, it
		 * will return #kArRunloopStopped from the run() API. If multiple runs of the runloop
		 * are nested, only the innermost will be stopped. If the runloop is calling out to a perform function
		 * or a handler callback, it will only be stopped when the callback returns.
		 *
		 * @retval #Ar::Status::success The runloop was stopped, or was already stopped.
		 */
		Ar::Status stop();

		void wake();

		/*!
		 * @brief Invoke a function on a runloop.
		 *
		 * The function will be called in FIFO order the next time the runloop runs. If the runloop is
		 * asleep, it will be woken and run immediately.
		 *
		 * This API can be called from any execution context.
		 *
		 * @param function The function to invoke on the runloop.
		 * @param param Arbitrary parameter passed to the function when it is called.
		 *
		 * @retval #Ar::Status::success The runloop was stopped, or was already stopped.
		 * @retval #Ar::Status::invalidParameterError The _function_ parameter was nullptr.
		 * @retval #kArQueueFullError No room to enqueue the function.
		 */
		Ar::Status perform(RunloopFunction function, void *param = 0);

		/*!
		 * @brief Associate a timer with a runloop.
		 *
		 * If the timer is already associated with another runloop, its association will be changed.
		 *
		 * @param timer The timer to associate with the runloop.
		 *
		 * @retval #Ar::Status::success The timer was added to the runloop.
		 * @retval #Ar::Status::invalidParameterError The _timer_ parameter was nullptr.
		 */
		Ar::Status addTimer(Timer &timer);

		/*!
		 * @brief Add a queue to a runloop.
		 *
		 * If the queue is already associated with another runloop, the #kArAlreadyAttachedError error
		 * is returned.
		 *
		 * @param queue The queue to associate with the runloop.
		 * @param callback Optional callback to handler an item received on the queue. May be nullptr.
		 * @param param Arbitrary parameter passed to the _callback_ when it is called.
		 *
		 * @retval #Ar::Status::success The timer was added to the runloop.
		 * @retval #kArAlreadyAttachedError A queue can only be added to one runloop at a time.
		 * @retval #Ar::Status::invalidParameterError The _queue_ parameter was nullptr.
		 */
		Ar::Status addQueue(Queue &queue, RunloopQueueHandler callback = nullptr, void *param = nullptr);

		/*!
		 * @brief Return the current runloop.
		 *
		 * @return The runloop currently running on the active thread. If no runloop is running, then nullptr
		 *      will be returned.
		 */
		static Runloop *getCurrent(void);

		List &getQueues() {return m_queues;}
	};

} // namespace Ar
#endif