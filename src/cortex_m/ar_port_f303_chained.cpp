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

#include "ar_port.hpp"
#include "argon/ar_kernel.hpp"
#include <cstring>
#include <iterator>
#include <exception>

#include "clock.h"
#include "device.h"
#include "ar_config.hpp"

namespace Ar {

	// prototype for fcn in assembly
	extern "C" std::uint32_t ar_port_yield_isr(std::uint32_t topOfStack, std::uint32_t isExtendedFrame);

// uTIM counts microseconds between system ticks, overflow increments sTIM
#define uTIM			TIM3
#define uTIM_IRQHandler TIM3_IRQHandler
	static constexpr auto F_uTIM = F_TIM3;
	static constexpr auto uTIM_IRQn = TIM3_IRQn;
	static constexpr auto uTIM_MAX = UINT16_MAX;
	static constexpr auto uTIM_CLKFZ_MSK = DBGMCU_APB1_FZ_DBG_TIM3_STOP;
	static inline void uTIM_CLK_EN() {
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		(void)RCC->APB1ENR;
	}
	static constexpr auto uTIM_PERIOD = 1_ms;

// sTIM counts system ticks
#define sTIM			TIM2
#define sTIM_IRQHandler TIM2_IRQHandler
	static constexpr auto F_sTIM = F_TIM2;
	static constexpr auto sTIM_IRQn = TIM2_IRQn;
	static constexpr auto sTIM_MAX = UINT32_MAX;
	static constexpr auto sTIM_CLKFZ_MSK = DBGMCU_APB1_FZ_DBG_TIM2_STOP;
	static inline void sTIM_CLK_EN() {
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		(void)RCC->APB1ENR;
	}

	//------------------------------------------------------------------------------
	// Prototypes
	//------------------------------------------------------------------------------

	std::uint32_t Port::yield_isr(std::uint32_t topOfStack, std::uint32_t isExtendedFrame);

	//------------------------------------------------------------------------------
	// Code
	//------------------------------------------------------------------------------

	void Port::init_system() {
		if constexpr (__FPU_USED) {
			// Enable FPU on Cortex-M4F.

			// Enable full access to coprocessors 10 and 11 to enable FPU access.
			SCB->CPACR |= (3 << 20) | (3 << 22);

			// Disable lazy FP context save.
			FPU->FPCCR |= FPU_FPCCR_ASPEN_Msk;
			FPU->FPCCR &= ~FPU_FPCCR_LSPEN_Msk;
		}

		std::set_terminate(Port::halt);

		// Init PSP.
		__set_PSP(reinterpret_cast<std::uint32_t>(Kernel::idleThread()->m_stackPointer));

		// Set priorities for the exceptions we use in the kernel.
		NVIC_SetPriority(SVCall_IRQn, handlerPriority);
		NVIC_SetPriority(PendSV_IRQn, handlerPriority);

		NVIC_EnableIRQ(sTIM_IRQn);
		NVIC_SetPriority(sTIM_IRQn, handlerPriority);
		NVIC_EnableIRQ(uTIM_IRQn);
		NVIC_SetPriority(uTIM_IRQn, handlerPriority);
	}

	void Port::init_timer() {
		// se corresponding bits in DBGMCU->APBxFZ to stop timer in debug, useful for stepping
		sTIM_CLK_EN();
		uTIM_CLK_EN();
		if constexpr (DEBUG) { DBGMCU->APB1FZ |= uTIM_CLKFZ_MSK | sTIM_CLKFZ_MSK; } // stop clocks to timer when core halted
		// NVIC_ClearPendingIRQ
		uTIM->CR1 = 0;
		uTIM->CR2 = (2 << TIM_CR2_MMS_Pos); // UPDATE EVENT as TRGO
		uTIM->PSC = (F_uTIM / 1000000UL) - 1; // psc to have 1Mhz (1us) ticking
		uTIM->ARR = uTIM_PERIOD.in_us() - 1; // warning, 16bit tmr
		uTIM->EGR = TIM_EGR_UG; // apply config
		uTIM->SR = ~TIM_SR_UIF;

		sTIM->CR1 = 0;
		sTIM->CR2 = 0;
		sTIM->SMCR = (TIM_SMCR_TS_1) | (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2); // count on TRGI, ITR2 (TIM3)
		sTIM->PSC = 0;
		sTIM->ARR = sTIM_MAX;
		sTIM->EGR = TIM_EGR_UG; // apply config
		sTIM->SR = ~TIM_SR_UIF;

		Port::set_wakeup_time(std::nullopt);

		bit::set(uTIM->CR1, TIM_CR1_CEN);
		bit::set(sTIM->CR1, TIM_CR1_CEN);
	}

	struct TimePair {
		std::uint32_t s;
		std::uint16_t u;
		TimePair() = delete;
		constexpr TimePair(std::uint32_t s_, std::uint16_t u_) : s(s_), u(u_) {}
	};

	std::optional<TimePair> targetWakeupTime {};

	enum class WakeInterruptSource { STIM, UTIM, UNKNOWN };

	volatile std::uint32_t g_scnt = 0;
	volatile std::uint32_t g_ucnt = 0;
	volatile WakeInterruptSource g_source = WakeInterruptSource::UNKNOWN;

	volatile std::uint8_t fastDispatch = 0;

	volatile std::uint8_t lastAction = 0;

	void disable_all_tim_interrupts() {
		// disable cmp interrupt generation
		bit::clear(sTIM->DIER, TIM_DIER_CC1IE);
		bit::clear(uTIM->DIER, TIM_DIER_CC1IE);
		// clear interrupt flags from timers
		sTIM->SR = 0;
		uTIM->SR = 0;
		// clear possibly pending interrupts
		NVIC_ClearPendingIRQ(sTIM_IRQn);
		NVIC_ClearPendingIRQ(uTIM_IRQn);

		sTIM->CCR1 = 0;
		uTIM->CCR1 = 0;
	}

	void handlePossibleWakeupInterrupt(WakeInterruptSource source) {
		g_source = source;
		if (fastDispatch) {
			lastAction = 11;
			volatile auto temp = fastDispatch;
			fastDispatch = 0;
			disable_all_tim_interrupts();
			targetWakeupTime.reset();
			Kernel::timerIsr();
			(void)temp;
			return;
		}
		assert(targetWakeupTime.has_value()); // otherwise, there was no reason why irq fired
		const auto now = Time::now();
		const auto times = targetWakeupTime.value();
		switch (source) {
			case WakeInterruptSource::STIM:
				lastAction = 12;
				// (most probably) first stage of multistage wakeup
				bit::clear(sTIM->DIER, TIM_DIER_CC1IE); // no more sTIm ticks
				sTIM->CCR1 = 0;

				// configure uTIM for final stage of timing
				uTIM->CCR1 = times.u;
				uTIM->SR = 0; // clear all irq flags
				bit::set(uTIM->DIER, TIM_DIER_CC1IE);
				if (uTIM->CNT >= uTIM->CCR1) {
					lastAction = 13;
					// we may have missed cmp event for irq generation
					disable_all_tim_interrupts();
					targetWakeupTime.reset();
					Kernel::timerIsr();
					return;
				}
				break;

			case WakeInterruptSource::UTIM: {
				// second stage of two stage wakeup or first stage of single stage wakeup
				auto sCnt = sTIM->CNT;
				auto uCnt = uTIM->CNT;
				g_scnt = sCnt;
				g_ucnt = uCnt;
				lastAction = 14;
				if (uCnt < uTIM->CCR1) {
					// already overflown
					lastAction = 15;
					sCnt = sTIM->CNT - 1; // reread sCnt but take overflow into account
					// __BKPT(0);
				}
				if (sCnt == times.s) { // sTIM already reached target wakeup value
					// so this is our final uTIM IRQ
					bit::clear(uTIM->DIER, TIM_DIER_CC1IE);
					disable_all_tim_interrupts();
					targetWakeupTime.reset();
					Kernel::timerIsr();
					return;
				} else {
					// assert(sCnt < times.s);
					assert();
					// more uTIM ticks (probably only 1) is yet to come
				}
			} break;

			default: assert();
		}
	}

	extern "C" void sTIM_IRQHandler(void) {
		sTIM->SR = 0; // clear all interrupt flags, we use CC1IF only
		handlePossibleWakeupInterrupt(WakeInterruptSource::STIM);
	}

	extern "C" void uTIM_IRQHandler(void) {
		uTIM->SR = 0; // clear all interrupt flags, we use CC1IF only
		handlePossibleWakeupInterrupt(WakeInterruptSource::UTIM);
	}

	//! Schedules next interrupt from timer.
	//!
	//! @param enables/disables timer
	//! @param configures delay of next interrupt from timer
	//! @return real value of delay, because clamping may occur
	//!     returns 0 when timer does not run or is delay:_us is 0
	//!     and interrupt is fired up immediately
	void Port::set_wakeup_time(std::optional<Time> time_) {
		// TODO: rethink if this is necessary
		lastAction = 100;
		disable_all_tim_interrupts();
		lastAction = 110;
		fastDispatch = 0;

		if (time_.has_value()) {
			const auto time = time_.value(); 
			TimePair times(time.in_us() / uTIM_PERIOD.in_us(), time.in_us() % uTIM_PERIOD.in_us());
			targetWakeupTime = times;

			auto now = Time::now();

			// wake is in past, fire IRQ immediately
			if (time <= now) {
				fastDispatch = 3;
				lastAction = 3;
				NVIC_SetPendingIRQ(uTIM_IRQn);
				return;
			}

			// ok, wakeup is in future (at leastfor snapshotted "now" value)
			if (times.s == sTIM->CNT) {
				// wakeup should happen in this whole sTIM tick
				// __BKPT(0);
				lastAction = 101;
				uTIM->CCR1 = times.u;
				uTIM->SR = 0; // clear all irq flags
				bit::set(uTIM->DIER, TIM_DIER_CC1IE);
				if (uTIM->CNT >= uTIM->CCR1 || times.s != sTIM->CNT) {
					// we may have missed cmp event for irq generation
					// or sTIM overflown (eg. on S:999 -> S+1:000) (we'd be 1 sTIM tick late)
					lastAction = 1;
					fastDispatch = 1;
					NVIC_SetPendingIRQ(uTIM_IRQn); // manually set pending
				}
			} else {
				lastAction = 102;
				// it's worth to use two step wakeup (sTIM->uTIM->wake)
				sTIM->CCR1 = times.s;
				bit::set(sTIM->DIER, TIM_DIER_CC1IE);

				// we may have configured tim too late and possibly missed interrupt opportunity
				if (sTIM->CNT >= sTIM->CCR1) {
					lastAction = 5;
					bit::clear(sTIM->DIER, TIM_DIER_CC1IE);
					sTIM->SR = 0;
					NVIC_SetPendingIRQ(sTIM_IRQn); // make sure, that irq fires
					return;
				}
			}

			// recheck if we did not miss target time
			if (time <= Time::now()) {
				disable_all_tim_interrupts();
				fastDispatch = 2;
				lastAction = 2;
				NVIC_SetPendingIRQ(uTIM_IRQn);
				return;
			}
		} else {
			lastAction = 6;
			disable_all_tim_interrupts(); // just don't fire interrupts on compare
			targetWakeupTime.reset();
		}
	}

	std::uint64_t Port::get_time_absolute_us() {
		std::uint16_t us_preread = uTIM->CNT;
		std::uint32_t ticks = sTIM->CNT;
		std::uint16_t us = uTIM->CNT;
		if (us_preread > us) { // overflow occurred during reading
			ticks = sTIM->CNT;
		}
		return static_cast<std::uint64_t>(ticks) * 1000 + us;
	}
} // namespace Ar

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
