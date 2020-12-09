#if !defined(_AR_QUEUE_H_)
	#define _AR_QUEUE_H_

	#include "argon/list.hpp"
	#include "argon/common.hpp"
	#include "argon/units.hpp"
	#include "argon/runloop.hpp"
	#include <optional>
	#include <array>
	#include <atomic>

namespace Ar {
	/*!
	 * @brief A blocking queue for inter-thread messaging.
	 *
	 * @ingroup ar_queue
	 */
	class Queue {
		friend Runloop;
		Name m_name; //!< Name of the queue.
		std::uint8_t *const m_elements; //!< Pointer to element storage.
		const std::size_t m_elementSize; //!< Number of bytes occupied by each element.
		const std::size_t m_capacity; //!< Maximum number of elements the queue can hold.
		std::atomic<std::size_t> m_head {0}; //!< Index of queue head.
		std::atomic<std::size_t> m_tail {0}; //!< Index of queue tail.
		std::atomic<std::size_t> m_count {0}; //!< Current number of elements in the queue.
		List m_sendBlockedList {}; //!< List of threads blocked waiting to send.
		List m_receiveBlockedList {}; //!< List of threads blocked waiting to receive data.
		Runloop *m_runLoop {nullptr}; //!< Runloop the queue is bound to.
		List::Node m_runLoopNode; //!< List node for the runloop's queue list.
		Runloop::RunloopQueueHandler m_runLoopHandler {nullptr}; //!< Handler function.
		void *m_runLoopHandlerParam {nullptr}; //!< User parameter for handler function.
	#if AR_GLOBAL_OBJECT_LISTS
		List::Node m_createdNode; //!< Created list node.
	#endif // AR_GLOBAL_OBJECT_LISTS
	  public:
		//! @brief Default constructor.
		Queue() = delete;

		//! @brief Constructor.
		//!
		//! @param name The new queue's name.
		//! @param storage Pointer to a buffer used to store queue elements. The buffer must be at least
		//!     @a elementSize * @a capacity bytes big.
		//! @param elementSize Size in bytes of each element in the queue.
		//! @param capacity The number of elements that the buffer pointed to by @a storage will hold.
		Queue(const char *name, void *storage, std::size_t elementSize, std::size_t capacity);

		//! @brief Queue cleanup.
		~Queue();

		//! @brief Get the queue's name.
		const char *getName() const { return m_name.data(); }

		//! @brief Add an item to the queue.
		//!
		//! The caller will block if the queue is full.
		//!
		//! @param element Pointer to the element to post to the queue. The element size was specified
		//!     in the init() call.
		//! @param timeout The maximum number of milliseconds that the caller is willing to wait in a
		//!     blocked state before the element can be sent. If this value is 0, or #kArNoTimeout, then
		//!     this method will return immediately if the queue is full. Setting the timeout to
		//!     #kArInfiniteTimeout will cause the thread to wait forever for a chance to send.
		//!
		//! @retval #Status::success
		//! @retval #kArQueueFullError
		Status send(const void *element, std::optional<Duration> timeout = std::nullopt);

		//! @brief Remove an item from the queue.
		//!
		//! @param[out] element
		//! @param timeout The maximum number of milliseconds that the caller is willing to wait in a
		//!     blocked state before an element is received. If this value is 0, or #kArNoTimeout, then
		//!     this method will return immediately if the queue is empty. Setting the timeout to
		//!     #kArInfiniteTimeout will cause the thread to wait forever for receive an element.
		//!
		//! @retval #Status::success
		//! @retval #kArQueueEmptyError
		Status receive(void *element, std::optional<Duration> timeout = std::nullopt);

		//! @brief Returns whether the queue is currently empty.
		bool isEmpty() const { return m_count == 0; }

		//! @brief Returns the current number of elements in the queue.
		std::size_t getCount() const { return m_count; }

	  private:
		//! @brief Disable copy constructor.
		Queue(const Queue &other);

		//! @brief Disable assignment operator.
		Queue &operator=(const Queue &other);

		Status send_internal(const void *element, std::optional<Duration> timeout = std::nullopt);

		constexpr auto QUEUE_ELEMENT(std::size_t index) { return &m_elements[m_elementSize * index]; }

		static void deferred_send(void *object, void *object2);
	};

	/*!
	 * @brief Template class to help statically allocate a Queue.
	 *
	 * @ingroup ar_queue
	 *
	 * This template class helps create a Queue instance by defining a static array of queue elements.
	 * The array length is one of the template parameters.
	 *
	 * Example of creating a queue and adding an element:
	 * @code
	 *      // MyQueueType holds up to five uint32_t elements.
	 *      typedef StaticQueue<uint32_t, 5> MyQueueType;
	 *
	 *      MyQueueType q; // Can statically allocate MyQueueType!
	 *      q.init("my queue");
	 *
	 *      uint32_t element = 512;
	 *      q.send(element);
	 *
	 *      q.send(1024);
	 *
	 *      Status s;
	 *
	 *      s = q.receive(&element);
	 *
	 *      element = q.receive(&s);
	 * @endcode
	 *
	 * @param T The queue element type.
	 * @param N Maximum number of elements the queue will hold.
	 */
	template<typename T, unsigned N>
	class StaticQueue : public Queue {
		using super = Queue;

	  public:
		using super::super;
		//! @brief Default constructor.
		StaticQueue() = delete;
		StaticQueue(const char *name, T *storage, std::size_t capacity) = delete;

		//! @brief Constructor.
		StaticQueue(const char *name) : super(name, m_storage, sizeof(T), N) {}

		//! @brief Alternate form of typed receive.
		//!
		//! @param[out] resultStatus The status of the receive operation is placed here.
		//!     May be NULL, in which case no status is returned.
		//! @param timeout Maximum time in ticks to wait for a queue element.
		T receive(std::optional<Duration> timeout = std::nullopt, Status *resultStatus = nullptr) {
			T element;
			Status status = Queue::receive(reinterpret_cast<void *>(&element), timeout);
			if (resultStatus) { *resultStatus = status; }
			return element;
		}

	  protected:
		std::array<T, N> m_storage; //!< Static storage for the queue elements.

	  private:
		//! @brief Disable copy constructor.
		StaticQueue(const StaticQueue<T, N> &other);

		//! @brief Disable assignment operator.
		StaticQueue &operator=(const StaticQueue<T, N> &other);
	};
} // namespace Ar
#endif