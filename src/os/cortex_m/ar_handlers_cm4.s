/*
 * Copyright (c) 2013-2014 Immo Software
 * All rights reserved.
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

// EXC_RETURN value to return to Thread mode, while restoring state from PSP.
EXC_RETURN  equ 0xfffffffd

/* specify the section where this code belongs */
            
        section .text:CODE
        thumb

        import ar_kernel_yield_isr

        public SVC_Handler
        public PendSV_Handler
        
SVC_Handler
PendSV_Handler
        
        // Get PSP
        mrs     r0, psp
        
        // Save registers on the stack and update the stack pointer (r0).
        stmdb   r0!, {r4-r11}
        
        // Invoke scheduler. On return, r0 contains the stack pointer for the new thread.
        ldr     r1, =ar_kernel_yield_isr
        blx     r1
        
        // Unstack saved registers.
        ldmia   r0!, {r4-r11}
        
        // Update PSP with new stack pointer.
        msr     psp, r0
        
        // Exit handler. Using a bx to the special EXC_RETURN values causes the
        // processor to perform the exception return behavior.
        ldr     r0, =EXC_RETURN
        bx      r0


        end
