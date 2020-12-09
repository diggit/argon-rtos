#if !defined(_AR_TRACE_H_)
	#define _AR_TRACE_H_

	#include "argon/common.hpp"
	#include "argon/config.hpp"
	#include "argon/port.hpp"
	#include <cstdint>

namespace Ar::Trace {

	//! All events start with a 32-bit message on ITM port 31, composed of an event ID in the
	//! top 8 bits plus a 24-bit value. Certain events are also followed by a 32-bit value in
	//! ITM port 30.

	enum class Event : std::uint8_t {
		SWITCHED = 1, //!< 2 value: 0=previous thread's new state, 1=new thread id
		CREATED = 2, //!< 2 value: 0=unused, 1=new thread id
		DELETED = 3, //!< 2 value: 0=unused, 1=deleted thread id
	};

	static void trace_init() {
		if constexpr (DEBUG) { Port::trace_init(); }
	}
	static void trace_1(Event eventID, std::uint32_t data) {
		if constexpr (DEBUG) { Port::trace_1(Ar::to_underlying(eventID), data); }
	}
	static void trace_2(Event eventID, std::uint32_t data0, void *data1) {
		if constexpr (DEBUG) { Port::trace_2(Ar::to_underlying(eventID), data0, data1); }
	}
} // namespace Ar::Trace

#endif