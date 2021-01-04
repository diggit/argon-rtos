/*
 * Copyright (c) 2013-2015 Immo Software
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
 * @file arport.h
 * @ingroup ar_port
 * @brief Header for the Argon RTOS.
 */

#if !defined(_AR_PORT_H_)
	#define _AR_PORT_H_

	#include "device.hpp"
	#include "argon/units.hpp"
	#include "argon/config.hpp"
	#include <cstdint>
	#include <type_traits>
	#include <optional>

namespace Ar {

	class Thread;

	namespace Port {

		//! Signature value written to the top of each thread's the stack. The scheduler looks
		//! for this value every time it activates a thread and halts if it is missing.
		static constexpr std::uint32_t stackCheckValue = 0xdeadbeef;

		//! Value to fill the stack with for detection of max stack usage. All bytes of this
		//! fill pattern must be the same.
		static constexpr std::uint32_t stackFillValue = 0xbabababa;

		//! Initial thread register values.
		static constexpr std::uint32_t initialxPSR = 0x01000000u; //!< Set T bit.
		static constexpr std::uint32_t initialLR = 0u; //!< Set to 0 to stop stack crawl by debugger.

		//! @brief Priorities for kernel exceptions.
		//! All handlers use the same, lowest priority.
		static constexpr std::uint8_t handlerPriority = 0xff;

		// these must be implemented by specific port
		//! @name Porting
		//@{
		void init_system();
		void init_timer();
		void set_wakeup_time(std::optional<Time> time);
		// static std::uint32_t get_time_absolute_ticks();
		// static std::uint32_t get_time_absolute_ms();
		std::uint64_t get_time_absolute_us();
		void prepare_stack(Thread *thread, std::uint32_t stackSize, void *param);
		void refill_stack(Thread *thread);
		void service_call();

		//! @brief Returns true if in IRQ state.
		static bool get_irq_state(void) { return __get_IPSR() != 0; }

		//! @brief Returns the number of milliseconds per tick.

		extern "C" std::uint32_t yield_isr(std::uint32_t topOfStack, std::uint32_t isExtendedFrame);

		void trace_init();
		void trace_1(std::uint8_t eventID, std::uint32_t data);
		void trace_2(std::uint8_t eventID, std::uint32_t data0, void *data1);

		//! @addtogroup ar_port
		//! @{

		//------------------------------------------------------------------------------
		// Definitions
		//------------------------------------------------------------------------------

		/*!
		 * @brief ARM Cortex-M specific thread struct fields.
		 */
		struct ThreadDataNonFPU {
			constexpr bool hasExtendedFrame() const { return false; }
			constexpr void setExtendedFrame(bool value) {};
		};

		struct ThreadDataFPU {
			bool m_hasExtendedFrame; //!< Whether the thread context has an extended stack frame with saved FP registers.
			bool hasExtendedFrame() const { return m_hasExtendedFrame; }
			void setExtendedFrame(bool value) { m_hasExtendedFrame = value; }
		};

		struct ThreadData : public std::conditional_t<__FPU_USED, ThreadDataFPU, ThreadDataNonFPU> {
			using super = std::conditional_t<__FPU_USED, ThreadDataFPU, ThreadDataNonFPU>;
			using super::super;
		};

		//! @}

		//! @addtogroup ar_port
		//! @{

		/*!
		 * @brief Context for a thread saved on the stack.
		 */
		struct ThreadContext {
			///! @name Stacked manually by us
			//@{
			std::uint32_t r4; //!< [SP+0] Lowest address on stack
			std::uint32_t r5; //!< [SP+4]
			std::uint32_t r6; //!< [SP+8]
			std::uint32_t r7; //!< [SP+12]
			std::uint32_t r8; //!< [SP+16]
			std::uint32_t r9; //!< [SP+20]
			std::uint32_t r10; //!< [SP+24]
			std::uint32_t r11; //!< [SP+28]
			//@}
			//! @name Stacked automatically by Cortex-M hardware
			//@{
			std::uint32_t r0; //!< [SP+32]
			std::uint32_t r1; //!< [SP+36]
			std::uint32_t r2; //!< [SP+40]
			std::uint32_t r3; //!< [SP+44]
			std::uint32_t r12; //!< [SP+48]
			std::uint32_t lr; //!< [SP+52]
			std::uint32_t pc; //!< [SP+56]
			std::uint32_t xpsr; //!< [SP+60] Highest address on stack
			//@}
		};

		//! @brief Stop the CPU because of a serious error.
		inline static void halt() { __BKPT(0); }

		//! @}
	} // namespace Port

} // namespace Ar

#endif // _AR_PORT_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
