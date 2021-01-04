#include "argon/port.hpp"

#include "argon/thread.hpp"
#include "argon/kernel.hpp"
#include "trace.hpp"

extern "C" void _exit(int) { Ar::Port::halt(); for(;;); }

namespace Ar::Port {

	//------------------------------------------------------------------------------
	// Variables
	//------------------------------------------------------------------------------

	//! @brief Global used solely to pass info back to asm PendSV handler code.
	extern "C" { bool g_ar_hasExtendedFrame = false; }

	//! A total of 64 bytes of stack space is required to hold the initial
	//! thread context.
	//!
	//! The entire remainder of the stack is filled with the pattern 0xba
	//! as an easy way to tell what the high watermark of stack usage is.
	void prepare_stack(Thread *thread, std::uint32_t stackSize, void *param) {

		// Clear the extended frame flag.
		thread->m_portData.setExtendedFrame(false);

		// 8-byte align stack.
		std::uint32_t sp = reinterpret_cast<std::uint32_t>(thread->m_stackBottom) + stackSize;
		std::uint32_t delta = sp & 7;
		sp -= delta;
		stackSize = (stackSize - delta) & ~7;
		thread->m_stackTop = reinterpret_cast<std::uint32_t *>(sp);
		thread->m_stackBottom = reinterpret_cast<std::uint32_t *>(sp - stackSize);

		if constexpr (config::THREAD_STACK_PATTERN_FILL) {
			std::memset(thread->m_stackBottom, stackFillValue & 0xff, stackSize);
			// Fill the stack with a pattern. We just take the low byte of the fill pattern since
			// memset() is a byte fill. This assumes each byte of the fill pattern is the same.
		}

		// Save new top of stack. Also, make sure stack is 8-byte aligned.
		sp -= sizeof(ThreadContext);
		thread->m_stackPointer = reinterpret_cast<uint8_t *>(sp);

		// Set the initial context on stack.
		ThreadContext *context = reinterpret_cast<ThreadContext *>(sp);
		context->xpsr = initialxPSR;
		context->pc = reinterpret_cast<std::uint32_t>(Thread::wrapper);
		context->lr = initialLR;
		context->r0 = reinterpret_cast<std::uint32_t>(thread); // Pass pointer to Thread object as first argument.
		context->r1 = reinterpret_cast<std::uint32_t>(param); // Pass arbitrary parameter as second argument.

		// For debug builds, set registers to initial values that are easy to identify on the stack.

		if constexpr (DEBUG) {
			context->r2 = 0x22222222;
			context->r3 = 0x33333333;
			context->r4 = 0x44444444;
			context->r5 = 0x55555555;
			context->r6 = 0x66666666;
			context->r7 = 0x77777777;
			context->r8 = 0x88888888;
			context->r9 = 0x99999999;
			context->r10 = 0xaaaaaaaa;
			context->r11 = 0xbbbbbbbb;
			context->r12 = 0xcccccccc;
		}

		// Write a check value to the bottom of the stack.
		*thread->m_stackBottom = stackCheckValue;
	}

	// refills unused space in stack by fill pattern
	void refill_stack(Thread *thread) {

		if constexpr (config::THREAD_STACK_PATTERN_FILL) {
			if (!thread) { return; }
			assert(thread->m_state != Thread::ThreadState::running);
			assert(thread != Kernel::getCurrent());

			const auto bottom = reinterpret_cast<volatile std::uint8_t *>(thread->m_stackBottom +1);
			// + 1; to preserve stackCheckValue
			const auto sp = reinterpret_cast<volatile std::uint8_t *>(thread->m_stackPointer - 1);
			// thread->m_stackPointer - 1 to avoid touching stack pointer
			assert(sp >= bottom); // stack overflow!
			std::memset(const_cast<std::uint8_t*>(bottom), stackFillValue & 0xff, sp-bottom);
			// Fill the stack with a pattern. We just take the low byte of the fill pattern since
			// memset() is a byte fill. This assumes each byte of the fill pattern is the same.
		}
	}

	extern "C" std::uint32_t yield_isr(std::uint32_t topOfStack, std::uint32_t isExtendedFrame) {

		// Save whether there is an extended frame.
		auto thread = Kernel::getCurrent();
		if (thread) { thread->m_portData.setExtendedFrame(isExtendedFrame); }

		// Run the scheduler.
		const std::uint32_t stack = Kernel::yieldIsr(topOfStack);

		// Pass whether there is an extended frame back to the asm code.
		g_ar_hasExtendedFrame = Kernel::getCurrent()->m_portData.hasExtendedFrame();

		return stack;
	}

	void service_call() {
		if constexpr (DEBUG) {
			assert(Kernel::lockCount() == 0);
			SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
			__DSB();
			__ISB();
		}
	}

	void trace_init() {}

	//! @brief Send one trace event word via ITM port 31.
	void trace_1(std::uint8_t eventID, std::uint32_t data) {
		if (ITM->TCR & ITM_TCR_ITMENA_Msk) {
			// Wait until we can send the event.
			while (!ITM->PORT[31].u32) {}

			// Event consists of 8-bit event ID plus 24-bits of event data.
			ITM->PORT[31].u32 = (static_cast<uint32_t>(eventID) << 24) | (data & 0x00ffffff);
		}
	}

	//! @brief Send a 2-word trace event via ITM ports 31 and 30.
	void trace_2(std::uint8_t eventID, std::uint32_t data0, void *data1) {
		if (ITM->TCR & ITM_TCR_ITMENA_Msk) {
			// Wait until we can send the event.
			while (!ITM->PORT[31].u32) {}

			// Event consists of 8-bit event ID plus 24-bits of event data.
			ITM->PORT[31].u32 = (static_cast<uint32_t>(eventID) << 24) | (data0 & 0x00ffffff);

			// Wait until we can send the event.
			while (!ITM->PORT[30].u32) {}

			// Send second data on port 30.
			ITM->PORT[30].u32 = reinterpret_cast<uint32_t>(data1);
		}
	}
} // namespace Ar::Port