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

#include "argon/config.hpp"
#include "argon/port.hpp"
#include "argon/kernel.hpp"
#include <cstdint>
#include <cstring>
#include <iterator>
#include <exception>
#include <ratio>

#include "argon/units.hpp"
#include "xhal/clock.hpp"
#include "xhal/utils.hpp"
#include "device.hpp"

using namespace xhal::utils;

namespace Ar {

	// prototype for fcn in assembly
	extern "C" std::uint32_t ar_port_yield_isr(std::uint32_t topOfStack, std::uint32_t isExtendedFrame);

// lsTIM counts microseconds between system ticks, overflow increments hsTIM
#define lsTIM			TIM3
#define lsTIM_IRQHandler TIM3_IRQHandler
	static constexpr auto lsTIM_bits = 16;
	static constexpr auto lsTIM_MAX = bit::fillFromLSB<std::uint32_t>(lsTIM_bits);
	static constexpr auto lsTIM_Freq = getPeripheralClock(xhal::stm32::clock::pTIM3);
	static constexpr auto lsTIM_IRQn = TIM3_IRQn;
	static constexpr auto lsTIM_CLKFZ_MSK = DBGMCU_APB1_FZ_DBG_TIM3_STOP;
	static inline void lsTIM_CLK_EN() {
		bit::set(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
		(void)RCC->APB1ENR;
	}

// hsTIM counts system ticks
#define hsTIM			TIM2
#define hsTIM_IRQHandler TIM2_IRQHandler
	static constexpr auto hsTIM_bits = 32;
	static constexpr auto hsTIM_MAX = bit::fillFromLSB<std::uint32_t>(hsTIM_bits);
	// static constexpr auto hsTIM_Freq = getPeripheralClock(xhal::stm32::clock::pTIM2);
	static constexpr auto hsTIM_IRQn = TIM2_IRQn;
	static constexpr auto hsTIM_CLKFZ_MSK = DBGMCU_APB1_FZ_DBG_TIM2_STOP;
	static inline void hsTIM_CLK_EN() {
		bit::set(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
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
			bit::set(SCB->CPACR, (3 << 20) | (3 << 22));

			// Disable lazy FP context save.
			bit::set(FPU->FPCCR, FPU_FPCCR_ASPEN_Msk);
			bit::clear(FPU->FPCCR, FPU_FPCCR_LSPEN_Msk);
		}

		std::set_terminate(Port::halt);

		// Init PSP.
		__set_PSP(reinterpret_cast<std::uint32_t>(Kernel::idleThread().m_stackPointer));

		// Set priorities for the exceptions we use in the kernel.
		// NVIC_SetPriority(SVCall_IRQn, handlerPriority); //unused at this moment
		NVIC_SetPriority(PendSV_IRQn, handlerPriority);

		NVIC_EnableIRQ(hsTIM_IRQn);
		NVIC_SetPriority(hsTIM_IRQn, handlerPriority);
		NVIC_EnableIRQ(lsTIM_IRQn);
		NVIC_SetPriority(lsTIM_IRQn, handlerPriority);
	}

	void Port::init_timer() {
		// se corresponding bits in DBGMCU->APBxFZ to stop timer in debug, useful for stepping
		hsTIM_CLK_EN();
		lsTIM_CLK_EN();
		#if !defined(NDEBUG)
		bit::set(DBGMCU->APB1FZ, lsTIM_CLKFZ_MSK | hsTIM_CLKFZ_MSK);// stop clocks to timer when core halted
		#endif
		// NVIC_ClearPendingIRQ
		lsTIM->CR1 = 0;
		lsTIM->CR2 = (2 << TIM_CR2_MMS_Pos); // UPDATE EVENT as TRGO
		lsTIM->PSC = lsTIM_Freq.in<std::mega>() - 1; // psc to have 1Mhz (1us) ticking
		lsTIM->ARR = lsTIM_MAX;
		lsTIM->EGR = TIM_EGR_UG; // apply config
		lsTIM->SR = ~TIM_SR_UIF;

		hsTIM->CR1 = 0;
		hsTIM->CR2 = 0;
		hsTIM->SMCR = (TIM_SMCR_TS_1) | (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2); // count on TRGI, ITR2 (TIM3)
		hsTIM->PSC = 0;
		hsTIM->ARR = hsTIM_MAX;
		hsTIM->EGR = TIM_EGR_UG; // apply config
		hsTIM->SR = ~TIM_SR_UIF;

		Port::set_wakeup_time(std::nullopt);

		bit::set(lsTIM->CR1, TIM_CR1_CEN);
		bit::set(hsTIM->CR1, TIM_CR1_CEN);
	}

	template<std::uint32_t splitInterval>
	struct TimePair_template {
		std::uint32_t s;
		std::uint16_t u;
		constexpr TimePair_template() = delete;
		constexpr TimePair_template(const Ar::units::Time &time) : s(time.in<std::micro>() / splitInterval), u(time.in<std::micro>() % splitInterval) {}
		
		template<typename Traw>
		constexpr TimePair_template(const Traw& raw) : s(raw / splitInterval), u(raw % splitInterval) {}
	};

	using TimePair = TimePair_template<lsTIM_MAX+1>;

	std::optional<TimePair> targetWakeupTime {};

	enum class WakeInterruptSource { HS_TIM, LS_TIM, UNKNOWN };

	volatile std::uint32_t g_scnt = 0;
	volatile std::uint32_t g_ucnt = 0;
	volatile WakeInterruptSource g_source = WakeInterruptSource::UNKNOWN;

	volatile std::uint8_t fastDispatch = 0;

	volatile std::uint8_t lastAction = 0;

	void disable_all_tim_interrupts() {
		// disable cmp interrupt generation
		bit::clear(hsTIM->DIER, TIM_DIER_CC1IE);
		bit::clear(lsTIM->DIER, TIM_DIER_CC1IE);
		// clear interrupt flags from timers
		hsTIM->SR = 0;
		lsTIM->SR = 0;
		// clear possibly pending interrupts
		NVIC_ClearPendingIRQ(hsTIM_IRQn);
		NVIC_ClearPendingIRQ(lsTIM_IRQn);

		hsTIM->CCR1 = 0;
		lsTIM->CCR1 = 0;
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
		const auto times = targetWakeupTime.value();
		switch (source) {
			case WakeInterruptSource::HS_TIM:
				lastAction = 12;
				// (most probably) first stage of multistage wakeup
				bit::clear(hsTIM->DIER, TIM_DIER_CC1IE); // no more hsTIm ticks
				hsTIM->CCR1 = 0;

				// configure lsTIM for final stage of timing
				lsTIM->CCR1 = times.u;
				lsTIM->SR = 0; // clear all irq flags
				bit::set(lsTIM->DIER, TIM_DIER_CC1IE);
				if (lsTIM->CNT >= lsTIM->CCR1) {
					lastAction = 13;
					// we may have missed cmp event for irq generation
					disable_all_tim_interrupts();
					targetWakeupTime.reset();
					Kernel::timerIsr();
					return;
				}
				break;

			case WakeInterruptSource::LS_TIM: {
				// second stage of two stage wakeup or first stage of single stage wakeup
				auto sCnt = hsTIM->CNT;
				auto uCnt = lsTIM->CNT;
				g_scnt = sCnt;
				g_ucnt = uCnt;
				lastAction = 14;
				if (uCnt < lsTIM->CCR1) {
					// already overflown
					lastAction = 15;
					sCnt = hsTIM->CNT - 1; // reread sCnt but take overflow into account
					// __BKPT(0);
				}
				if (sCnt == times.s) { // hsTIM already reached target wakeup value
					// so this is our final lsTIM IRQ
					bit::clear(lsTIM->DIER, TIM_DIER_CC1IE);
					disable_all_tim_interrupts();
					targetWakeupTime.reset();
					Kernel::timerIsr();
					return;
				} else {
					// assert(sCnt < times.s);
					assert(false);
					// more lsTIM ticks (probably only 1) is yet to come
				}
			} break;

			default: assert(false);
		}
	}

	//! Schedules next interrupt from timer.
	//!
	//! @param enables/disables timer
	//! @param configures delay of next interrupt from timer
	//! @return real value of delay, because clamping may occur
	//!     returns 0 when timer does not run or is delay:_us is 0
	//!     and interrupt is fired up immediately
	void Port::set_wakeup_time(std::optional<Time> time_) {
		lastAction = 100;
		// TODO: rethink if this is necessary
		disable_all_tim_interrupts();
		lastAction = 110;
		fastDispatch = 0;

		if (time_.has_value()) {
			const auto time = time_.value();

			TimePair times(time.in<std::micro>());
			targetWakeupTime = times;

			auto now = Time::now();

			// wake is in past, fire IRQ immediately
			if (time <= now) {
				fastDispatch = 3;
				lastAction = 3;
				NVIC_SetPendingIRQ(lsTIM_IRQn);
				return;
			}

			// ok, wakeup is in future (at leastfor snapshotted "now" value)
			if (times.s == hsTIM->CNT) {
				// wakeup should happen in this whole hsTIM tick
				// __BKPT(0);
				lastAction = 101;
				lsTIM->CCR1 = times.u;
				lsTIM->SR = 0; // clear all irq flags
				bit::set(lsTIM->DIER, TIM_DIER_CC1IE);
				if (lsTIM->CNT >= lsTIM->CCR1 || times.s != hsTIM->CNT) {
					// we may have missed cmp event for irq generation
					// or hsTIM overflown (eg. on S:999 -> S+1:000) (we'd be 1 hsTIM tick late)
					lastAction = 1;
					fastDispatch = 1;
					NVIC_SetPendingIRQ(lsTIM_IRQn); // manually set pending
				}
			} else {
				lastAction = 102;
				// it's worth to use two step wakeup (hsTIM->lsTIM->wake)
				hsTIM->CCR1 = times.s;
				bit::set(hsTIM->DIER, TIM_DIER_CC1IE);

				// we may have configured tim too late and possibly missed interrupt opportunity
				if (hsTIM->CNT >= hsTIM->CCR1) {
					lastAction = 5;
					bit::clear(hsTIM->DIER, TIM_DIER_CC1IE);
					hsTIM->SR = 0;
					NVIC_SetPendingIRQ(hsTIM_IRQn); // make sure, that irq fires
					return;
				}
			}

			// recheck if we did not miss target time
			if (time <= Time::now()) {
				disable_all_tim_interrupts();
				fastDispatch = 2;
				lastAction = 2;
				NVIC_SetPendingIRQ(lsTIM_IRQn);
				return;
			}
		} else {
			lastAction = 6;
			disable_all_tim_interrupts(); // just don't fire interrupts on compare
			targetWakeupTime.reset();
		}
	}

	std::uint64_t Port::get_time_absolute_us() {
		const auto ls_preread = lsTIM->CNT;
		auto hs = hsTIM->CNT;
		const auto ls = lsTIM->CNT;
		if (ls_preread > ls) { // overflow occurred during reading
			hs = hsTIM->CNT;
		}
		return (static_cast<std::uint64_t>(hs) << lsTIM_bits) + ls;
		// return (static_cast<std::uint64_t>(ls) * lsTIM_PERIOD.in<std::micro>()) + hs;
	}
} // namespace Ar

void hsTIM_IRQHandler(void) {
	hsTIM->SR = 0; // clear all interrupt flags, we use CC1IF only
	Ar::handlePossibleWakeupInterrupt(Ar::WakeInterruptSource::HS_TIM);
}

void lsTIM_IRQHandler(void) {
	lsTIM->SR = 0; // clear all interrupt flags, we use CC1IF only
	Ar::handlePossibleWakeupInterrupt(Ar::WakeInterruptSource::LS_TIM);
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
