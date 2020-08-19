#if !defined(_AR_ATOMIC_H_)
	#define _AR_ATOMIC_H_

#include <cstdint>

namespace Ar::Atomic {
	//! @addtogroup ar_atomic
	//! @{

	//! @name Atomic operations
	//@{
	/*!
	 * @brief Atomic 8-bit add operation.
	 *
	 * A memory barrier is performed prior to the add operation.
	 *
	 * @param value Pointer to the word to add to.
	 * @param delta Signed value to atomically add to *value.
	 * @return The original value is returned.
	 */
	// extern "C" std::int8_t add8(volatile std::int8_t *value, std::int8_t delta);

	/*!
	 * @brief Atomic 16-bit add operation.
	 *
	 * A memory barrier is performed prior to the add operation.
	 *
	 * @param value Pointer to the word to add to.
	 * @param delta Signed value to atomically add to *value.
	 * @return The original value is returned.
	 */
	// extern "C" std::int16_t add16(volatile std::int16_t *value, std::int16_t delta);

	/*!
	 * @brief Atomic 32-bit add operation.
	 *
	 * A memory barrier is performed prior to the add operation.
	 *
	 * @param value Pointer to the word to add to.
	 * @param delta Signed value to atomically add to *value.
	 * @return The original value is returned.
	 */
	// extern "C" std::int32_t add32(volatile std::int32_t *value, std::int32_t delta);

	/*!
	 * @brief Atomic 8-bit compare and swap operation.
	 *
	 * Tests the word pointed to by @a value for equality with @a expectedValue. If they are
	 * equal, the word pointed to by @a value is set to @a newValue. If *value is not
	 * equal to @a expectedValue, then no change is made. The return value indicates whether
	 * the swap was performed. Of course, this entire operation is guaranteed to be
	 * atomic even on multiprocessor platforms.
	 *
	 * A memory barrier is performed prior to the compare and swap operation.
	 *
	 * @param value Pointer to the word to compare and swap.
	 * @param expectedValue Value to compare against.
	 * @param newValue Value to value to swap in if *value is equal to expectedValue.
	 * @retval false No change was made to *value.
	 * @retval true The swap was performed, and *value is now equal to newValue.
	 */
	// extern "C" bool cas8(volatile std::int8_t *value, std::int8_t expectedValue, std::int8_t newValue);

	/*!
	 * @brief Atomic 16-bit compare and swap operation.
	 *
	 * Tests the word pointed to by @a value for equality with @a expectedValue. If they are
	 * equal, the word pointed to by @a value is set to @a newValue. If *value is not
	 * equal to @a expectedValue, then no change is made. The return value indicates whether
	 * the swap was performed. Of course, this entire operation is guaranteed to be
	 * atomic even on multiprocessor platforms.
	 *
	 * A memory barrier is performed prior to the compare and swap operation.
	 *
	 * @param value Pointer to the word to compare and swap.
	 * @param expectedValue Value to compare against.
	 * @param newValue Value to value to swap in if *value is equal to expectedValue.
	 * @retval false No change was made to *value.
	 * @retval true The swap was performed, and *value is now equal to newValue.
	 */
	// extern "C" bool cas16(volatile std::int16_t *value, std::int16_t expectedValue, std::int16_t newValue);

	/*!
	 * @brief Atomic 32-bit compare and swap operation.
	 *
	 * Tests the word pointed to by @a value for equality with @a expectedValue. If they are
	 * equal, the word pointed to by @a value is set to @a newValue. If *value is not
	 * equal to @a expectedValue, then no change is made. The return value indicates whether
	 * the swap was performed. Of course, this entire operation is guaranteed to be
	 * atomic even on multiprocessor platforms.
	 *
	 * A memory barrier is performed prior to the compare and swap operation.
	 *
	 * @param value Pointer to the word to compare and swap.
	 * @param expectedValue Value to compare against.
	 * @param newValue Value to value to swap in if *value is equal to expectedValue.
	 * @retval false No change was made to *value.
	 * @retval true The swap was performed, and *value is now equal to newValue.
	 */
	// extern "C" bool cas32(volatile std::int32_t *value, std::int32_t expectedValue, std::int32_t newValue);
	//@}

	//! @}
} // namespace Ar::Atomic
#endif