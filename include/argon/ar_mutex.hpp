#if !defined(_AR_MUTEX_H_)
	#define _AR_MUTEX_H_

	#include "argon/ar_thread.hpp"
	#include "ar_common.hpp"
	#include "ar_list.hpp"
	#include <array>
	#include <optional>
	#include <type_traits>
	#include <cstdint>

namespace Ar {

	/*!
	 * @brief Mutex object.
	 *
	 * @ingroup ar_mutex
	 *
	 * Very similar to a binary semaphore, except that a single thread can lock the mutex multiple times
	 * without deadlocking. In this case, the number of calls to get() and put() must be matched.
	 *
	 * Another difference from semaphores is that mutexes support priority inheritance. If a high-
	 * priority thread blocks on a mutex held by a lower-priority thread, the thread holding the mutex
	 * will have its priority temporarily raised to the priority of the thread waiting on the mutex.
	 *
	 * @see Mutex::Guard
	 */
	class Mutex {
	  public:
		Name m_name; //!< Name of the mutex.
		volatile Thread *m_owner; //!< Current owner thread of the mutex.
		volatile std::uint32_t m_ownerLockCount; //!< Number of times the owner thread has locked the mutex.
		std::uint8_t m_originalPriority; //!< Original priority of the owner thread before its priority was raised.
		List m_blockedList {Thread::sort_by_priority}; //!< List of threads blocked on the mutex.
	#if AR_GLOBAL_OBJECT_LISTS
		List::Node m_createdNode; //!< Created list node.
	#endif // AR_GLOBAL_OBJECT_LISTS

		//! @brief Default constructor.
		Mutex() = delete;

		//! @brief Constructor.
		Mutex(const char *name);

		//! @brief Cleanup.
		~Mutex();

		//! @brief Get the mutex's name.
		const char *getName() const { return m_name.data(); }

		//! @brief Lock the mutex.
		//!
		//! If the thread that already owns the mutex calls get() more than once, a count is incremented
		//! rather than attempting to decrement the underlying semaphore again. The converse is true for
		//! put(), thus allowing a thread to lock a mutex any number of times as long as there are
		//! matching get() and put() calls.
		//!
		//! @param timeout The maximum number of milliseconds that the caller is willing to wait in a
		//!     blocked state before the semaphore can be obtained. If this value is 0, or #kArNoTimeout,
		//!     then this method will return immediately if the lock cannot be obtained. Setting
		//!     the timeout to #kArInfiniteTimeout will cause the thread to wait forever for a chance to
		//!     get the lock.
		//!
		//! @retval #Ar::Status::success The mutex was obtained without error.
		//! @retval #kArTimeoutError The specified amount of time has elapsed before the mutex could be
		//!     obtained.
		//! @retval #kArObjectDeletedError Another thread deleted the semaphore while the caller was
		//!     blocked on it.
		Ar::Status get(std::optional<Duration> timeout = std::nullopt);

		//! @brief Unlock the mutex.
		//!
		//! Only the owning thread is allowed to unlock the mutex. If the owning thread has called get()
		//! multiple times, it must also call put() the same number of time before the underlying
		//! semaphore is actually released. It is illegal to call put() when the mutex is not owned by
		//! the calling thread.
		//!
		//! @retval #kArAlreadyUnlockedError The mutex is not locked.
		//! @retval #kArNotOwnerError The caller is not the thread that owns the mutex.
		Ar::Status put();

		static void deferred_get(void *object, void *object2);
		static void deferred_put(void *object, void *object2);

		//! @brief Returns the current owning thread, if there is one.
		Thread *getOwner() { return const_cast<Thread *>(m_owner); }

		//! @brief Returns whether the mutex is currently locked.
		//!
		//! @retval true The mutex is locked.
		//! @retval false The mutex is unlocked.
		bool isLocked() { return m_ownerLockCount != 0; }

		/*!
		 * @brief Utility class to automatically get and put a mutex.
		 *
		 * @ingroup ar_mutex
		 *
		 * This class is intended to be stack allocated. It gets and holds a mutex for the
		 * duration of the scope in which it is declared. Once it goes out of scope, the destructor
		 * automatically puts the lock.
		 */
		class Guard {
		  public:
			//! @brief Constructor which gets the mutex.
			Guard(Mutex &mutex, std::optional<Duration> timeout = std::nullopt) : m_mutex(mutex) { m_mutex.get(timeout); }

			//! @brief Destructor that puts the mutex.
			~Guard() { m_mutex.put(); }

		  protected:
			Mutex &m_mutex; //!< The mutex to hold.
		};

	  private:
		//! @brief Disable copy constructor.
		Mutex(const Mutex &other);

		//! @brief Disable assignment operator.
		Mutex &operator=(const Mutex &other);

		Ar::Status put_internal();
		Ar::Status get_internal(std::optional<Duration> timeout = std::nullopt);
	};

} // namespace Ar
#endif