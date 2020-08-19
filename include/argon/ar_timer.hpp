#if !defined(_AR_TIMER_H_)
	#define _AR_TIMER_H_

	#include "ar_list.hpp"
	#include "ar_units.hpp"
	#include "ar_common.hpp"
	#include <optional>

namespace Ar {

	struct Runloop;

	/*!
	 * @brief Timer object.
	 *
	 * @ingroup ar_timer
	 */
	class Timer {
	  public:
		//! @ingroup ar_timer
		using Entry = void (*)(Timer *timer, void *param);

		//! @brief Timer callback function that takes an instance of this class.
		using Callback = void (*)(Timer *timer, void *param);

		//! @brief Modes of operation for timers.
		//!
		//! @ingroup ar_timer
		enum class Mode {
			oneShotTimer, //!< Timer fires a single time.
			periodicTimer //!< Timer repeatedly fires every time the interval elapses.
		};

		Name m_name {}; //!< Name of the timer.
		List::Node m_activeNode; //!< Node for the list of active timers.
	#if AR_GLOBAL_OBJECT_LISTS
		ar_list_node_t m_createdNode; //!< Created list node.
	#endif // AR_GLOBAL_OBJECT_LISTS
		Timer::Entry m_callback; //!< Timer expiration callback.
		void *m_param; //!< Arbitrary parameter for the callback.
		Mode m_mode; //!< One-shot or periodic mode.
		bool m_isActive {false}; //!< Whether the timer is running and on the active timers list.
		bool m_isRunning {false}; //!< Whether the timer callback is executing.
		Duration m_delay; //!< Delay in ticks.
		Time m_wakeupTime; //!< Expiration time in ticks.
		std::optional<Time> m_deferredWakeupTime {}; //!< used to pass deferredWakeup value from deferred ection
		Runloop *m_runLoop {}; //!< Runloop to which the timer is bound.

		//! @brief Default constructor.
		Timer() = delete;

		//! @brief Constructor.
		Timer(const char *name, Callback callback, void *param, Timer::Mode timerMode, Duration delay);

		//! @brief Destructor.
		~Timer();

		//! @brief Get the timer's name.
		const char *getName() const { return m_name.data(); }

		//! @brief Start the timer running.
		Status start();

		//! @brief Stop the timer.
		Status stop();

		//! @brief Returns whether the timer is currently running.
		bool isActive() const { return m_isActive; }

		//! @brief Adjust the timer's delay.
		Status setDelay(Duration delay);

		//! @brief Get the current delay for the timer.
		Duration getDelay() const { return m_delay; }

		//! @brief Sort timer list by ascending wakeup time.
		static bool sort_by_wakeup(List::Node *a, List::Node *b);
		//@}

		static void run_timers(List & timersList);

	  protected:
		Callback m_userCallback; //!< The user timer callback.

		//! @brief Converts the timer struct to an instance of this class.
		static void timer_wrapper(Timer *timer, void *arg);

	  private:
		//! @brief Disable copy constructor.
		Timer(const Timer &other);

		//! @brief Disable assignment operator.
		Timer &operator=(const Timer &other);

		Status start_internal(Time wakeupTime);
		Status stop_internal();

		static void deferred_start(void *object, void *object2);
		static void deferred_stop(void *object, void *object2);
	}; // namespace Ar

	/*!
	 * @brief Timer object taking a member function callback.
	 *
	 * @ingroup ar_timer
	 */
	template<class T>
	class TimerWithMemberCallback : public Timer {
	  public:
		//! @brief Timer callback function that takes an instance of this class.
		using Callback = void (T::*)(Timer *timer);

		//! @brief Default constructor.
		TimerWithMemberCallback<T>() = delete;

		//! @brief Constructor taking a member function callback.
		TimerWithMemberCallback<T>(const char *name, T *object, Callback callback, Timer::Mode timerMode, Duration delay) :
			Timer(name, callback, object, timerMode, delay){}

		//! @brief Destructor.
		~TimerWithMemberCallback<T>();

	  protected:
		Callback m_memberCallback; //!< The user timer callback.

		//! @brief Call the member function.
		void invoke(T *obj) { (obj->*m_memberCallback)(this); }

		//! @brief Template function to invoke a callback that is a member function.
		static void member_callback(Timer *timer, void *param) {
			TimerWithMemberCallback<T> *_this = static_cast<TimerWithMemberCallback<T> *>(timer);
			T *obj = static_cast<T *>(param);
			_this->invoke(obj);
		}

	  private:
		//! @brief Disable copy constructor.
		TimerWithMemberCallback<T>(const TimerWithMemberCallback<T> &other);

		//! @brief Disable assignment operator.
		TimerWithMemberCallback<T> &operator=(const TimerWithMemberCallback<T> &other);
	};
} // namespace Ar
#endif