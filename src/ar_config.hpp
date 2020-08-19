/*
 * Copyright (c) 2013-2018 Immo Software
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
 * @file ar_config.h
 * @ingroup ar
 * @brief Configuration settings for the Argon RTOS.
 */

#if !defined(_AR_CONFIG_H_)
	#define _AR_CONFIG_H_

	#include "ar_units.hpp"
	#include <cstdint>

namespace Ar::config {

	//------------------------------------------------------------------------------
	// Definitions
	//------------------------------------------------------------------------------

	//! @addtogroup ar_config
	//! @{

	//! @page Configuration
	//! @ingroup ar_config
	//!
	//! These configuration macros are used to control features of Argon. There are several ways
	//! to modify the configuration. The recommended method is to copy the `ar_config.h` file to
	//! your application's source code tree and modify it. Be sure to place the new config file's
	//! directory earlier in the include paths than the Argon source directory.
	//! <br/><br/>
	//! Another option for adjusting configuration settings is to specify the configuration macro
	//! values directly on the compiler's command line, in order to override the default values of
	//! these macros.
	//! <br/><br/>
	//! Some configuration macros have defaults based on the debug or release build type. For these,
	//! the value of the DEBUG macro is used to determine the build type.

	//! @brief The string to use for the name of an object that wasn't provided a name.
	// static constexpr auto ANONYMOUS_OBJECT_NAME = "<anon>";

	//! @brief Set to 1 to enable the lists of all created kernel objects.
	//!
	//! Default is enabled for debug builds, disabled for release builds.
	static constexpr bool GLOBAL_OBJECT_LISTS = (DEBUG);

	//! @name Idle thread config
	//@{

	//! @brief Controls whether the idle thread puts the processor to sleep until the next interrupt.
	//!
	//! Set to 1 to enable. The default is to disable idle sleep for debug builds, enabled for
	//! release builds.
	static constexpr bool ENABLE_IDLE_SLEEP = !(DEBUG);

	//! @brief Size in bytes of the idle thread's stack.
	static constexpr std::size_t IDLE_THREAD_STACK_SIZE = (200);

	//@}

	//! @name System load config
	//@{

	//! @brief When set to true, total system CPU load will be computed.
	static constexpr bool ENABLE_SYSTEM_LOAD = true;

	//! @brief When set to true, per-thread and total system CPU load will be computed.
	//! implicitly enables ENABLE_SYSTEM_LOAD
	static constexpr bool ENABLE_SYSTEM_LOAD_PER_THREAD = true;

	//! @brief Microsecond period over which CPU load is computed.
	//!
	//! The nominal sampler period is 1 second.
	static constexpr ::Ar::units::Duration SYSTEM_LOAD_SAMPLE_PERIOD = 1_s;

	//@}

	//! @brief Whether to fill a new thread's stack with a pattern.
	//!
	//! Filling the stack with a pattern enables one to determine maximum stack usage of a thread.
	//! The downside is that it takes longer to initialize a thread. Default is enabled for debug
	//! builds, disabled for release.
	static constexpr bool THREAD_STACK_PATTERN_FILL = (DEBUG);

	//! @name Main thread config
	//@{

	//! @brief Set to 1 to cause main() to be run in a thread.
	//!
	//! Enabling this option will cause the kernel to automatically start prior to main() being
	//! called. A thread is created for you with the entry point set to main(). The main thread's
	//! priority is set with the #AR_MAIN_THREAD_PRIORITY macro. The stack size is determined by
	//! a combination of the linker file and #AR_SCHEDULER_STACK_SIZE.
	static constexpr bool ENABLE_MAIN_THREAD = true;

	//! @brief Size in bytes of the stack used by the scheduler and interrupts if the main thread
	//! is enabled.
	//!
	//! This size is subtracted from the C stack size specified by the linker file. The remainder
	//! is used for the main thread itself.
	static constexpr std::size_t SCHEDULER_STACK_SIZE = (256);

	//! @brief Priority for the main thread.
	static constexpr auto MAIN_THREAD_PRIORITY = (128);

	//@}

	//! @brief Set to 1 to enable tickless idle.
	static constexpr bool ENABLE_TICKLESS_IDLE = true;

	//! @brief Maximum number of actions deferred from IRQ context.
	static constexpr auto DEFERRED_ACTION_QUEUE_SIZE = 8;

	//! @brief Maximum number of functions queued in a run loop.
	static constexpr auto RUNLOOP_FUNCTION_QUEUE_SIZE = 8;

	//! @brief Enable runtime checking of linked lists.
	//!
	//! Normally not required.
	static constexpr bool ENABLE_LIST_CHECKS = false;

	//! @brief Enable kernel event tracing.
	static constexpr bool AR_ENABLE_TRACE = (DEBUG);

	//! @brief Configure sleeping time resolution.
	static constexpr auto timeQuanta = 1_ms;

	//! @brief Configure scheduler period.
	static constexpr auto schedulerPeriod = 10;

	//! @brief Enable asserts.
	static constexpr bool AR_ENABLE_ASSERT = (DEBUG);

	//! @brief Maximum name length.
	static constexpr std::size_t MAX_NAME_LENGTH = 8;

	//! @}
} // namespace Ar::config

#endif // _AR_CONFIG_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
