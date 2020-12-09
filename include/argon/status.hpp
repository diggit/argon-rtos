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

	const std::array<const char, 7> anon_name {"<anon>"};

	using Name = std::array<char, config::MAX_NAME_LENGTH>;

	template<typename E>
	constexpr auto to_underlying(E e) noexcept {
		return static_cast<std::underlying_type_t<E>>(e);
	}

	namespace bit {
	#if __cplusplus > 201703L
		// C++20 std::type_identity_t

		template<typename T>
		constexpr inline void set(T &value, std::type_identity_t<T> mask) {
			value = value | mask;
		}

		template<typename T>
		constexpr inline void clear(T &value, std::type_identity_t<T> mask) {
			value = value & ~mask;
		}

		template<typename T>
		constexpr inline T mask(T value, std::type_identity_t<T> mask) {
			return value & mask;
		}

		template<typename T>
		constexpr inline void modify(T &value, std::type_identity_t<T> mask, std::type_identity_t<T> data) {
			value = (value & ~mask) | data;
		}
	#endif

		template<std::uint8_t bitOffset, typename T>
		constexpr inline T bit() {
			static_assert(std::is_integral_v<T>, "T must be integral type");
			static_assert(sizeof(T) * 8 > bitOffset, "bit offset must fit in T");
			return (static_cast<T>(1)) << bitOffset;
		}

		template<typename T = std::uint32_t>
		constexpr inline T bit(std::uint8_t bitOffset) {
			static_assert(std::is_integral_v<T>, "T must be integral type");
			return (static_cast<T>(1)) << bitOffset;
		}

		// this is epic idea... http://stackoverflow.com/a/7919546
		// #define FILL_BITS_FROM_LSB(bits) ((1 << (bits)) - 1)
		template<typename T>
		constexpr T fillFromLSB(std::uint8_t bits) {
			static_assert(std::is_integral_v<T>, "T must be integral type");
			static_assert(not std::is_signed_v<T>, "T must be unsigned type");
			if (bits >= (sizeof(T) * 8)) { return -1; }
			return ((static_cast<T>(1)) << bits) - 1;
		}
		static_assert(fillFromLSB<std::uint32_t>(0) == 0, "fillFromLSB broken");
		static_assert(fillFromLSB<std::uint32_t>(9) == 0x000001FFUL, "fillFromLSB broken");
		static_assert(fillFromLSB<std::uint32_t>(32) == 0xFFFFFFFFUL, "fillFromLSB broken");
		static_assert(fillFromLSB<std::uint32_t>(33) == 0xFFFFFFFFUL, "fillFromLSB broken");

		// #define FILL_BITS_FROM_MSB(bits) (~((0x80 >> ((bits)-1)) - 1)) //meh, you get it?
		template<typename T>
		constexpr T fillFromMSB(std::uint8_t bits) {
			static_assert(std::is_integral_v<T>, "T must be integral type");
			static_assert(not std::is_signed_v<T>, "T must be unsigned type");
			if (bits >= (sizeof(T) * 8)) { return -1; }
			return ~fillFromLSB<T>((sizeof(T) * 8) - bits); // do not allow negative numbers
		}
		static_assert(fillFromMSB<std::uint32_t>(0) == 0, "fillFromMSB broken");
		static_assert(fillFromMSB<std::uint32_t>(9) == 0xFF800000UL, "fillFromMSB broken");
		static_assert(fillFromMSB<std::uint32_t>(32) == 0xFFFFFFFFUL, "fillFromMSB broken");
		static_assert(fillFromMSB<std::uint32_t>(33) == 0xFFFFFFFFUL, "fillFromMSB broken");

	} // namespace bit

	template<class T>
	T max(const T a, const T b) {
		return (a < b) ? b : a; // or: return comp(a,b)?b:a; for version (2)
	}

	template<class T>
	T min(const T a, const T b) {
		return (a > b) ? b : a; // or: return comp(a,b)?b:a; for version (2)
	}

	template<class T>
	T clamp(const T value_min, const T value_max, const T value) {
		return max(min(value, value_max), value_min);
	}

	template<class T>
	T uabs(const T a) {
		return (a > 0) ? a : -a; // or: return comp(a,b)?b:a; for version (2)
	}

	template<typename T>
	constexpr bool isOdd(T value) {
		static_assert(::std::is_integral_v<T>, "T must integral type");
		return bit::mask(value, 1);
	}

	template<typename T>
	constexpr bool isEven(T value) {
		static_assert(::std::is_integral_v<T>, "T must integral type");
		return !isOdd(value);
	}

	template<typename T>
	constexpr T abs(const T value) {
		// static_assert(std::is_signed_v<T>, "T must be signed, abs does not make sense on unsigned")
		if constexpr (std::is_signed_v<T>) {
			if (value < 0) { return -value; }
		}
		return value;
	}

	template<typename T>
	constexpr bool isPowerOf2(T value) {
		static_assert(std::is_integral_v<T>, "T must integral type");
		if (!value) { return 0; }
		T tmp = abs(value);
		return !(tmp & (tmp - 1));
	}

} // namespace Ar
#endif