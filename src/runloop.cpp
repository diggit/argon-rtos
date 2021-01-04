/*
 * Copyright (c) 2016-2018 Immo Software
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
 * @brief Source for Ar microkernel runloops.
 */

#include "argon/runloop.hpp"
#include "argon/thread.hpp"
#include "argon/timer.hpp"
#include "argon/queue.hpp"
#include "argon/kernel.hpp"
#include "argon/assert.hpp"
#include <cstring>
#include <optional>

namespace Ar {

	//------------------------------------------------------------------------------
	// Code
	//------------------------------------------------------------------------------

	// See ar_kernel.h for documentation of this function.
	Runloop::Runloop(const char *name) {

		assert(Ar::Port::get_irq_state());

		std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);

#if AR_GLOBAL_OBJECT_LISTS
		runloop->m_createdNode.m_obj = runloop;
		g_ar_objects.runloops.add(&runloop->m_createdNode);
#endif
	}

	// See ar_kernel.h for documentation of this function.
	Runloop::~Runloop() {
		assert(Ar::Port::get_irq_state());
		assert(m_isRunning);

#if AR_GLOBAL_OBJECT_LISTS
		g_ar_objects.runloops.remove(&runloop->m_createdNode);
#endif
	}

	Ar::Status Runloop::run(RunloopResult *object, std::optional<Ar::Duration> timeout) {
		// It's ok to nest runs of the runloop, but only on a single thread.
		auto thread = Thread::getCurrent();
		if (m_isRunning && m_thread != thread) { return Status::runLoopAlreadyRunningError; }
		// Another check to make sure no other runloop is already running on the current thread.
		if (thread->m_runLoop && thread->m_runLoop != this) { return Status::runLoopAlreadyRunningError; }
		// Disallow running from interrupt context.
		if (Ar::Port::get_irq_state()) { return Ar::Status::notFromInterruptError; }

		// Clear returned object.
		if (object) { object->m_queue = nullptr; }

		bool isNested = m_isRunning;
		if (!isNested) {
			// Associate this runloop with the current thread.
			thread->m_runLoop = this;
			m_thread = thread;

			// Clear stop flag.
			m_stop = false;

			// Set running flag.
			m_isRunning = true;
		}

		// Prepare timeout.
		auto startTime = Ar::Time::now();

		Ar::Status returnStatus = Status::runLoopStopped;

		// Run it.
		do {
			// Invoke timers.
			Timer::run_timers(m_timers);

			// Invoke one queued function.
			if (m_functionCount) {
				auto i = m_functionHead.load();

				RunloopFunctionInfo fn = m_functions[i];

				m_functionCount--;
				// This is the only line that modifies m_functionHead.
				m_functionHead = (i + 1) % Ar::config::RUNLOOP_FUNCTION_QUEUE_SIZE;

				assert(fn.function);
				fn.function(fn.param);
			}

			// Check pending queues.
			if (!m_queues.isEmpty()) {
				auto *queue = m_queues.getHead<Queue>();
				assert(queue);
				assert(false); //TODO. properly implement
				// if (queue->m_count < 2) { m_queues.remove(&queue->m_runLoopNode); }

				// if (queue->m_count > 0) {
				// 	if (queue->m_runLoopHandler) {
				// 		// Call out to run loop queue source handler.
				// 		queue->m_runLoopHandler(queue, queue->m_runLoopHandlerParam);
				// 	} else {
				// 		// No handler associated with this queue, so exit the run loop.
				// 		if (object) { object->m_queue = queue; }

				// 		returnStatus = Status::runLoopQueueReceived;
				// 		break;
				// 	}
				// }
			}

			// Check timeout. Adjust the timeout based on how long we've run so far.
			if (timeout.has_value()) {
				if(startTime.elapsed(timeout.value())) {
					// Timed out, exit runloop.
					returnStatus = Status::timeoutError;
					break;
				}
			}

			std::optional<Time> soonestWakeup;
			if (timeout) {
				soonestWakeup = startTime + timeout.value();
			}

			// Make sure we don't sleep past the next scheduled timer.
			if (!m_timers.isEmpty()) {
				auto timer = m_timers.getHead<Timer>();

				// Timers should always have a wakeup time in the future.
				assert(timer->m_wakeupTime >= Ar::Time::now());

				if (!soonestWakeup || timer->m_wakeupTime < soonestWakeup.value()) {
					soonestWakeup = timer->m_wakeupTime;
				}
			}

			// Don't sleep if there are queued functions or sources.
			if (!m_functionCount && m_queues.isEmpty()) {
				// Sleep the runloop's thread until nest event is scheduled (timer wakeup or timeout)
				if (soonestWakeup) { Thread::sleepUntil(soonestWakeup.value()); }
			}
		} while (!m_stop);

		if (!isNested) {
			// Clear associated thread.
			Thread::getCurrent()->m_runLoop = nullptr;
			m_thread = nullptr;

			// Clear running flag.
			m_isRunning = false;
		}

		// TODO return different status for timed out versus stopped?
		return returnStatus;
	}

	Ar::Status Runloop::stop() {

		// Set the stop flag.
		m_stop = true;

		// Wake the runloop in case it is blocked.
		wake();

		return Ar::Status::success;
	}

	void Runloop::wake() {
		if (m_isRunning && m_thread) { m_thread->resume(); }
	}

	Ar::Status Runloop::perform(RunloopFunction function, void *param) {
		if (!function) { return Ar::Status::invalidParameterError; }

		// TODO block if queue is full?
		auto count = m_functionCount.load();
		do {
			count = m_functionCount;
			if (count + 1 > config::RUNLOOP_FUNCTION_QUEUE_SIZE) { return Status::queueFullError; }
		}while(!m_functionCount.compare_exchange_weak(count, count + 1));

		auto last = m_functionTail.load();
		do {
			last = m_functionTail;
		}while(!m_functionTail.compare_exchange_weak(last, (last + 1) % config::DEFERRED_ACTION_QUEUE_SIZE));

		auto tail = last;

		RunloopFunctionInfo &fn = m_functions[tail];
		fn.function = function;
		fn.param = param;

		// Wake the runloop in case it is blocked.
		wake();

		return Ar::Status::success;
	}

	Ar::Status Runloop::addTimer(Timer &timer) {
		timer.m_runLoop = this;

		return Ar::Status::success;
	}

	Ar::Status Runloop::addQueue(Queue &queue, RunloopQueueHandler callback, void *param) {

		// A queue can only be added to one runloop at a time.
		if (queue.m_runLoop != NULL && queue.m_runLoop != this) { return Status::alreadyAttachedError; }

		// Set runloop on the queue.
		queue.m_runLoopHandler = callback;
		queue.m_runLoopHandlerParam = param;
		queue.m_runLoop = this;

		return Ar::Status::success;
	}

	Runloop *Runloop::getCurrent(void) { return Thread::getCurrent()->m_runLoop; }

} // namespace Ar

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
