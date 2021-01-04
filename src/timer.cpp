/*
 * Copyright (c) 2007-2018 Immo Software
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
 * @file
 * @brief Source for Ar microkernel timers.
 */

#include "argon/common.hpp"
#include "argon/timer.hpp"
#include "argon/kernel.hpp"
#include "argon/port.hpp"
#include "argon/assert.hpp"

#include <cstring>
#include <cstdint>

namespace Ar {

	//------------------------------------------------------------------------------
	// Code
	//------------------------------------------------------------------------------

	// See ar_kernel.h for documentation of this function.
	Timer::Timer(const char *name, Timer::Entry callback, void *param, Timer::Mode timerMode, Duration delay) :
		m_activeNode(this), m_callback(timer_wrapper), m_param(param), m_mode(timerMode), m_delay(delay), m_wakeupTime(Time::now()), m_userCallback(callback) {
		assert(callback && !delay.isZero());
		assert(!Ar::Port::get_irq_state());

		std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);

#if AR_GLOBAL_OBJECT_LISTS
		timer->m_createdNode.m_obj = timer;
		g_ar_objects.timers.add(&timer->m_createdNode);
#endif
	}

	// See ar_kernel.h for documentation of this function.
	Timer::~Timer() {
		assert(!Port::get_irq_state());

		stop();

#if AR_GLOBAL_OBJECT_LISTS
		g_ar_objects.timers.remove(&timer->m_createdNode);
#endif
	}

	//! @brief Handles starting or restarting a timer.
	Ar::Status Timer::start_internal(Ar::Time wakeupTime) {
		KernelLock guard;

		assert(m_runLoop);

		// Handle a timer that is already active.
		if (m_isActive) { m_runLoop->m_timers.remove(&m_activeNode); }

		m_wakeupTime = wakeupTime;
		m_isActive = true;

		m_runLoop->m_timers.add(&m_activeNode);

		// Wake runloop so it will recompute its sleep time.
		m_runLoop->wake();

		return Ar::Status::success;
	}

	void Timer::deferred_start(void *object, void *object2) {
		if (!assertWrap(object)) { return; };
		auto timer = reinterpret_cast<Timer *>(object);
		if (!assertWrap(timer->m_deferredWakeupTime.has_value())) {return;}
		timer->start_internal(timer->m_deferredWakeupTime.value());
	}

	// See ar_kernel.h for documentation of this function.
	Ar::Status Timer::start() {
		if (!m_runLoop) { return Status::timerNoRunloop; }

		// The callback should have been verified by the create function.
		assert(m_callback);

		auto wakeupTime = Time::now() + m_delay;

		// Handle irq state by deferring the operation.
		if (Port::get_irq_state()) {
			m_deferredWakeupTime = wakeupTime; // save for deferred execution
			return Kernel::postDeferredAction(Timer::deferred_start, this, nullptr);
		}

		return start_internal(wakeupTime);
	}

	Ar::Status Timer::stop_internal() {
		KernelLock guard;

		if (m_runLoop) {
			m_runLoop->m_timers.remove(&m_activeNode);

			// Wake runloop so it will recompute its sleep time.
			m_runLoop->wake();
		}

		// m_wakeupTime = 0; //TODO: is it necessary?
		m_isActive = false;

		return Ar::Status::success;
	}

	void Timer::deferred_stop(void *object, void *object2) {
		if (!assertWrap(object)) { return; };
		reinterpret_cast<Timer *>(object)->stop_internal();
	}

	// See ar_kernel.h for documentation of this function.
	Ar::Status Timer::stop() {
		if (!m_isActive) { return Status::timerNotRunningError; }
		if (!m_runLoop) { return Status::timerNoRunloop; }

		// Handle irq state by deferring the operation.
		if (Port::get_irq_state()) { return Kernel::postDeferredAction(Timer::deferred_stop, this); }

		return stop_internal();
	}

	// See ar_kernel.h for documentation of this function.
	Ar::Status Timer::setDelay(Duration delay) {
		if (delay.isZero()) { return Ar::Status::invalidParameterError; }

		m_delay = delay;

		// If the timer is running, we need to restart it, unless it is a periodic
		// timer whose callback is currently executing. In that case, the timer will
		// automatically be rescheduled for us, so if we did it here then it would be
		// rescheduled twice causing a double delay.
		if (m_isActive && !(m_isRunning && m_mode == Mode::periodicTimer)) { start(); }

		return Ar::Status::success;
	}

	//! @brief Execute callbacks for all expired timers.
	//!
	//! While a timer callback is running, the m_isRunning flag on the timer is set to true.
	//! When the callback returns, a one shot timer is stopped while a periodic timer is
	//! rescheduled based on its delay. If a periodic timer's callback runs so long that the next
	//! wakeup time is in the past, it will be rescheduled to a time in the future that is aligned
	//! with the period.
	void Timer::run_timers(List &timersList) {
		// Check if we need to handle a timer.
		if (timersList.m_head) {
			List::Node *timerNode = timersList.m_head;
			do {
				auto *timer = timerNode->getObject<Timer>();
				assert(timer);

				// Exit loop if all remaining timers on the list wake up in the future.
				if (timer->m_wakeupTime > Time::now()) { break; }

				// Invoke the timer callback.
				assert(timer->m_callback);
				timer->m_isRunning = true;
				timer->m_callback(timer, timer->m_param);
				timer->m_isRunning = false;

				// Check that the timer wasn't stopped in its callback.
				if (timer->m_isActive) {
					switch (timer->m_mode) {
						case Mode::oneShotTimer:
							// Stop a one shot timer after it has fired.
							timer->stop();
							break;

						case Mode::periodicTimer: {
							// Restart a periodic timer without introducing (much) jitter. Also handle
							// the cases where the timer callback ran longer than the next wakeup.
							auto wakeupTime = timer->m_wakeupTime + timer->m_delay;
							// in case we missed next wakeup time
							if (wakeupTime < Time::now()) {
								// Compute the delay to the next wakeup in the future that is aligned
								// to the timer's period. (skip tick but don't shift phase)
								const auto extrayDelay = timer->m_delay * (static_cast<int>((Time::now() - wakeupTime) / timer->m_delay) + 1);
								wakeupTime += extrayDelay;
							}
							timer->start_internal(wakeupTime);
							break;
						}
					}
				}

				timerNode = timerNode->m_next;
			} while (timerNode != timersList.m_head);
		}
	}

	//! @brief Sort timers ascending by wakeup time.
	bool Timer::sort_by_wakeup(List::Node *a, List::Node *b) {
		Timer *timerA = a->getObject<Timer>();
		Timer *timerB = b->getObject<Timer>();
		return (timerA->m_isActive && timerA->m_wakeupTime < timerB->m_wakeupTime);
	}

	void Timer::timer_wrapper(Timer *timer, void *arg) {
		Timer *_this = static_cast<Timer *>(timer);
		assert(_this);
		if (_this->m_userCallback) { _this->m_userCallback(_this, arg); }
	}

} // namespace Ar