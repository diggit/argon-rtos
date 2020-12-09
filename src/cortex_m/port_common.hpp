#if !defined(_AR_PORT_COMMON_H_)
	#define _AR_PORT_COMMON_H_

#include <cstdint>

namespace Ar::port {
	// when there is no other way and you really have to disable interrupt for short moment in some scope
	class IrqGuard {
	  private:
		// in fact we really need just one bit to store,
		std::uint32_t cpuSR; // occupies 4B on stack but saves 3 masking instruction
		// uint8_t cpuSR;  // occupies 1B on stack (depends on alignment of surrounding data), adds cca 2 masking instructions
	  public:
		IrqGuard(void) {
			asm("mrs   %[output], PRIMASK\n\t"
				"cpsid I\n\t"
				: [ output ] "=r"(cpuSR)::);
		}
		~IrqGuard(void) { asm("msr PRIMASK, %[input];\n\t" ::[input] "r"(cpuSR) :); };
	};
} // namespace Ar::port

#endif