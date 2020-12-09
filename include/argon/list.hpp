#if !defined(_AR_LIST_H_)
	#define _AR_LIST_H_

// #include "argon/thread.hpp"
// #include "argon/timer.hpp"
// #include "argon/queue.hpp"

namespace Ar {

	/*!
	 * @brief Linked list.
	 */
	struct List {

		//! @name Linked lists
		//@{
		/*!
		 * @brief Linked list node.
		 */
		struct Node {
			Node *m_next {nullptr}; //!< Next node in the list.
			Node *m_prev {nullptr}; //!< Previous node in the list.
			void *m_obj {nullptr}; //!< Pointer to the object on the list.

			constexpr Node() = default;
			constexpr Node(void *node) : m_obj(node) {};

			// Internal utility methods.

			//! @brief Convert the @a m_obj pointer to a particular type.
			template<typename T>
			T *getObject() {
				return reinterpret_cast<T *>(m_obj);
			}
			void insertBefore(Node *node); //!< @brief Insert this node before another node on the list.
		};

		//! @name Function types
		//@{
		//! Function type used for sorting object lists.
		using SortPredicate = bool (*)(Node *a, Node *b);

		// Intentionally not initialized, because static initialization of Kernel may happen later that threads initialization.
		// Threads created to start immediately inserts themselves into kernels lists.
		// So initializing kernel later would clear the head of list making it empty.
		// We rely on initialization
		Node *m_head; //!< Pointer to the head of the list. Will be NULL if the list is empty.
		SortPredicate m_predicate {nullptr}; //!< Sort predicate to use for this list. Items are added to the end if NULL.

		constexpr List(SortPredicate predicate = nullptr) : m_predicate(predicate) {}

		// Internal utility methods.
		//! @brief Convert the node head's @a m_obj pointer to a particular type.
		template<typename T>
		T *getHead() {
			return m_head ? m_head->getObject<T>() : nullptr;
		}

		inline bool isEmpty() const { return m_head == nullptr; } //!< @brief Return whether the list is empty.
		bool contains(Node *item); //!< @brief Return whether the list contains a given node.
		void add(Node *item); //!< @brief Add an item to the list.
		// inline void add(Thread *item); //!< @brief Add a thread to the list.
		// inline void add(Timer *item); //!< @brief Add a timer to the list.
		// inline void add(Queue *item); //!< @brief Add a queue to the list.
		void remove(Node *item); //!< @brief Remove an item from the list.
		// inline void remove(Thread *item); //!< @brief Remove a thread from the list.
		// inline void remove(Timer *item); //!< @brief Remove a timer from the list.
		// inline void remove(Queue *item); //!< @brief Remove a queue from the list.
		void check();
	};
	//@}
} // namespace Ar
#endif