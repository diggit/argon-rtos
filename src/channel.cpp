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
 * @brief Source for Ar microkernel channels.
 */

#include "argon/channel.hpp"
#include "argon/thread.hpp"
#include "argon/kernel.hpp"
#include "argon/assert.hpp"
#include "argon/units.hpp"
#include <cstring>

namespace Ar {

	//------------------------------------------------------------------------------
	// Code
	//------------------------------------------------------------------------------

	// See ar_kernel.h for documentation of this function.
	Channel::Channel(const char *name, std::size_t width) : m_width((width == 0) ? sizeof(void *) : width) {
		assert(!Port::get_irq_state());

		std::strncpy(m_name.data(), name ? name : Ar::anon_name.data(), Ar::config::MAX_NAME_LENGTH);

#if AR_GLOBAL_OBJECT_LISTS
		m_createdNode.m_obj = channel;
		g_ar_objects.channels.add(&m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	// See ar_kernel.h for documentation of this function.
	Channel::~Channel() {
		assert(!Port::get_irq_state());

		// Unblock all threads blocked on this channel.
		Thread *thread;
		while (m_blockedSenders.m_head) {
			thread = m_blockedSenders.getHead<Thread>();
			thread->unblockWithStatus(m_blockedSenders, Status::objectDeletedError);
		}

		while (m_blockedReceivers.m_head) {
			thread = m_blockedReceivers.getHead<Thread>();
			thread->unblockWithStatus(m_blockedReceivers, Status::objectDeletedError);
		}

#if AR_GLOBAL_OBJECT_LISTS
		g_ar_objects.channels.remove(&channel->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS
	}

	//! @brief Handles blocking a thread on a channel.
	//!
	//! The kernel must be locked prior to entry of this function.
	Ar::Status Channel::block(List &myDirList, void *value, std::optional<Duration> timeout) {
		// Nobody waiting, so we must block. Return immediately if the timeout is 0.
		if (timeout.has_value() && timeout.value().isZero()) { return Status::timeoutError; }

		// Block this thread on the channel. Save the value pointer into the thread
		// object so the other side of this channel can access it.
		auto thread = Thread::getCurrent();
		thread->m_channelData = value;
		thread->block(myDirList, timeout);

		// We're back from the scheduler. Check for errors and exit early if there was one.
		if (thread->m_unblockStatus != Ar::Status::success) {
			myDirList.remove(&thread->m_blockedNode);
			return thread->m_unblockStatus;
		}

		return Ar::Status::success;
	}

	Ar::Status Channel::send_receive_internal(bool isSending, List &myDirList, List &otherDirList, void *value, std::optional<Duration> timeout) {
		KernelLock guard;

		// Are there any blocked threads for the opposite direction of this call?
		if (otherDirList.isEmpty()) {
			return block(myDirList, value, timeout);
		} else {
			// Get the first thread blocked on this channel.
			auto *thread = otherDirList.getHead<Thread>();

			// Figure out the direction of the data transfer.
			void *src;
			void *dest;
			if (isSending) {
				src = value;
				dest = thread->m_channelData;
			} else {
				src = thread->m_channelData;
				dest = value;
			}

			// Do the transfer. Optimize word-sized channels so we don't have to call into memcpy().
			// This is dangerous, what about std::uint8_t[4] ? Diabled...
			// if (m_width == sizeof(uint32_t)) {
			// 	*(uint32_t *)dest = *(uint32_t *)src;
			// } else {
			std::memcpy(dest, src, m_width);
			// }

			// Unblock the other side.
			thread->unblockWithStatus(otherDirList, Ar::Status::success);
		}

		return Ar::Status::success;
	}

	void Channel::deferred_send(void *object, void *object2) {
		auto *channel = reinterpret_cast<Channel *>(object);
		if(!assertWrap(channel)) {return;}
		channel->send_receive_internal(true, channel->m_blockedSenders, channel->m_blockedReceivers, object2, Duration::zero());
	}

	//! @brief Common channel send/receive code.
	Ar::Status Channel::send_receive(bool isSending, List &myDirList, List &otherDirList, void *value, std::optional<Duration> timeout) {

		// Ensure that only 0 timeouts are specified when called from an IRQ handler.
		if (Port::get_irq_state()) {
			if (!isSending || (timeout.has_value() && timeout.value().isZero())) { return Ar::Status::notFromInterruptError; }

			// Handle irq state by deferring the operation.
			return Kernel::postDeferredAction(Channel::deferred_send, this, value);
		}

		return send_receive_internal(isSending, myDirList, otherDirList, value, timeout);
	}

	//------------------------------------------------------------------------------
	// EOF
	//------------------------------------------------------------------------------

} // namespace Ar