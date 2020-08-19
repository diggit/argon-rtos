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
 * @file ar_internal.h
 * @ingroup ar_port
 * @brief Header for the Argon RTOS.
 */

#if !defined(_AR_INTERNAL_H_)
	#define _AR_INTERNAL_H_

	#include "argon/argon.hpp"
	#include "ar_port.hpp"
	#include "ar_config.hpp"
	#include "ar_list.hpp"

	#include <cstdint>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

	#if !defined(WEAK)
		#define WEAK __attribute__((weak))
	#endif

	#if !defined(ALWAYS_INLINE)
		#define ALWAYS_INLINE __attribute__((always_inline))
	#endif

namespace Ar {

	//------------------------------------------------------------------------------
	// API
	//------------------------------------------------------------------------------


	//! @name Interrupt handlers
	//@{
	extern "C" void		ar_kernel_periodic_timer_isr(void);
	extern "C" uint32_t ar_kernel_yield_isr(uint32_t topOfStack);
	//@}


	// Inline list method implementation.
	// inline bool _ar_list::isEmpty() const { return m_head == NULL; }
	// inline void _ar_list::add(Thread *item) { add(&item->m_threadNode); }
	// inline void _ar_list::add(ar_timer_t *item) { add(&item->m_activeNode); }
	// inline void _ar_list::add(ar_queue_t *item) { add(&item->m_runLoopNode); }
	// inline void _ar_list::remove(Thread *item) { remove(&item->m_threadNode); }
	// inline void _ar_list::remove(ar_timer_t *item) { remove(&item->m_activeNode); }
	// inline void _ar_list::remove(ar_queue_t *item) { remove(&item->m_runLoopNode); }


#endif // _AR_INTERNAL_H_
	//------------------------------------------------------------------------------
	// EOF
	//------------------------------------------------------------------------------
