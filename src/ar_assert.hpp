#if !defined(_AR_ASSERT_H_)
	#define _AR_ASSERT_H_

	#include "ar_config.hpp"
	#include "ar_port.hpp"

namespace Ar {

	static inline bool assert(bool condition = false) {
		if constexpr (config::AR_ENABLE_ASSERT) {
			if (!condition) {
				Port::halt();
				for (;;) {};
			}
		}
		return condition;
	}

} // namespace Ar
#endif
