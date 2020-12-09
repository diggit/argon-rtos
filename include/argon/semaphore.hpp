#if !defined(_AR_SEMAPHORE_H_)
	#define _AR_SEMAPHORE_H_

	#include "argon/list.hpp"
	#include "argon/common.hpp"
	#include "argon/units.hpp"
	#include <optional>

namespace Ar {

	/*!
	 * @brief Counting semaphore class.
	 *
	 * @ingroup ar_sem
	 *
	 * @see Semaphore::Guard
	 */
	class Semaphore {
		Name m_name {}; //!< Name of the semaphore.
		volatile unsigned int m_count; //!< Current semaphore count. Value of 0 means the semaphore is owned.
		List m_blockedList {}; //!< List of threads blocked on the semaphore.
	#if AR_GLOBAL_OBJECT_LISTS
		List::Node m_createdNode; //!< Created list node.
	#endif // AR_GLOBAL_OBJECT_LISTS

	  public:
		//! @brief Constructor.
		//!
		//! @param name Pass a name for the semaphore. If NULL is passed the name will be set to an
		//!     empty string.
		//! @param count The initial semaphore count. Setting this value to 0 will cause the first call
		//!     to get() to block until put() is called. A value of 1 or greater will allow that many
		//!     calls to get() to succeed.
		//!
		Semaphore(const char *name = nullptr, unsigned int count = 0);

		//! @brief Destructor.
		//!
		//! Any threads on the blocked list will be unblocked immediately. Their return status from the
		//! get() method will be #kArObjectDeletedError.
		~Semaphore();

		//! @brief Get the semaphore's name.
		const char *getName() const { return m_name.data(); }

		//! @brief Acquire the semaphore.
		//!
		//! The semaphore count is decremented. If the count is 0 upon entering this method then the
		//! caller thread is blocked until the count reaches 1. Threads are unblocked in the order in
		//! which they were blocked. Priority is not taken into consideration, so priority inversions
		//! are possible.
		//!
		//! @note This function may be called from interrupt context only if the timeout parameter is
		//!     set to #kArNoTimeout (or 0).
		//!
		//! @param timeout The maximum number of milliseconds that the caller is willing to wait in a
		//!     blocked state before the semaphore can be obtained. If this value is 0, or #kArNoTimeout,
		//!     then this method will return immediately if the semaphore cannot be obtained. Setting
		//!     the timeout to #kArInfiniteTimeout will cause the thread to wait forever for a chance to
		//!     get the semaphore.
		//!
		//! @retval #Ar::Status::success The semaphore was obtained without error.
		//! @retval #kArTimeoutError The specified amount of time has elapsed before the semaphore could be
		//!     obtained.
		//! @retval #kArObjectDeletedError Another thread deleted the semaphore while the caller was
		//!     blocked on it.
		//! @retval #Ar::Status::notFromInterruptError A non-zero timeout is not alllowed from the interrupt
		//!     context.
		Ar::Status acquire(std::optional<Duration> timeout = std::nullopt);

		//! @brief Release the semaphore.
		//!
		//! The semaphore count is incremented.
		//!
		//! @note This call is safe from interrupt context.
		//!
		//! @retval #Ar::Status::success The semaphore was released without error.
		Ar::Status release();

		//! @brief Returns the current semaphore count.
		unsigned getCount() const { return m_count; }

	  protected:
		Ar::Status acquire_internal(std::optional<Duration> timeout);
		Ar::Status release_internal();

		static void deferred_acquire(void *object, void *object2);
		static void deferred_release(void *object, void *object2);

	  private:
		//! @brief Disable copy constructor.
		Semaphore(const Semaphore &other);

		//! @brief Disable assignment operator.
		Semaphore &operator=(const Semaphore &other);
	};

} // namespace Ar
#endif