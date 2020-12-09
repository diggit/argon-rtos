#ifndef _AR_COMMON_HDR_INCLUDED
	#define _AR_COMMON_HDR_INCLUDED

	#include <array>
	#include "argon/config.hpp"

namespace Ar {

	//! @brief Argon status and error codes.
	//!
	//! @ingroup ar
	enum class Status {
		success = 0, //!< Operation was successful.
		timeoutError, //!< Timeout while blocked on an object.
		objectDeletedError, //!< An object was deleted while a thread was blocked on it. This may be a semaphore, mutex, or queue.
		queueFullError, //!< The queue is at maximum capacity and cannot accept more elements.
		queueEmptyError, //!< No elements are in the queue.
		invalidPriorityError, //!< The requested thread priority is invalid.
		stackSizeTooSmallError, //!< The thread's stack size is too small.
		notFromInterruptError, //!< The requested operation cannot be performed from interrupt context.
		notOwnerError, //!< The caller is not the owning thread.
		alreadyUnlockedError, //!< The mutex is already unlocked.
		invalidParameterError, //!< An invalid parameter value was passed to the function.
		timerNotRunningError, //!< The timer is not running.
		timerNoRunloop, //!< The timer is not associated with a run loop.
		outOfMemoryError, //!< Allocation failed.
		invalidStateError, //!< The thread is an invalid state for the given operation.
		alreadyAttachedError, //!< The object is already attached to a runloop.
		runLoopAlreadyRunningError, //!< The runloop is already running on another thread.
		runLoopStopped, //!< The runloop was stopped.
		runLoopQueueReceived, //!< The runloop exited due to a value received on an associated queue.
	};

	//this simple wrapper class is workaround for missing operator bool() on Result_t
	class StatusW {
		Status value_;
	public:
		constexpr StatusW(Status r):	value_(r){};
		constexpr operator bool() const {return value_ == Status::success;}
		constexpr operator Status() const {return value_;}
		constexpr bool operator ==(StatusW rhs) const {return value_ == rhs.value_;}
		constexpr bool operator ==(Status rhs) const {return value_ == rhs;}
		constexpr bool operator !=(StatusW rhs) const {return value_ != rhs.value_;}
		constexpr bool operator !=(Status rhs) const {return value_ != rhs;}
		
		constexpr bool isOk() {return *this;};
		constexpr Status value(){return value_;}

		static constexpr StatusW OK(){return StatusW(Status::success);}

	};

	const std::array<const char, 7> anon_name {"<anon>"};

	using Name = std::array<char, config::MAX_NAME_LENGTH>;

	template<typename E>
	constexpr auto to_underlying(E e) noexcept {
		return static_cast<std::underlying_type_t<E>>(e);
	}

} // namespace Ar

#endif