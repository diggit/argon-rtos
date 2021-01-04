#include "argon/list.hpp"
#include "argon/assert.hpp"
#include "argon/config.hpp"
#include "argon/port.hpp"

namespace Ar {
	//! Updates the list node's links and those of @a node so that the object is inserted before
	//! @node in the list.
	//!
	//! @note This method does not update the list head. To actually insert a node into the list you
	//!     need to use List::add().
	void List::Node::insertBefore(List::Node *node) {
		m_next = node;
		m_prev = node->m_prev;
		node->m_prev->m_next = this;
		node->m_prev = this;
	}

	//! Performs a linear search of the linked list.
	bool List::contains(List::Node *item) {
		// Check if the node is even on a list.
		if (item->m_next == nullptr || item->m_prev == nullptr) { return false; }

		if (m_head) {
			List::Node *node = m_head;
			do {
				if (node == item) {
					// Matching node was found in the list.
					return true;
				}
				node = node->m_next;
			} while (node != m_head);
		}

		return false;
	}

	//! The item is inserted in either FIFO or sorted order, depending on whether the predicate
	//! member is set. If the predicate is nullptr, the new list item will be inserted at the
	//! end of the list, maintaining FIFO order.
	//!
	//! If the predicate function is provided, then it is used to search for the insert position on the
	//! list. The new item will be inserted before the first existing list item for which the predicate
	//! function returns true when called with its first parameter set to the new item and second
	//! parameter set to the existing item.
	//!
	//! The list is maintained as a doubly-linked circular list. The last item in the list has its
	//! next link set to the head of the list, and vice versa for the head's previous link. If there is
	//! only one item in the list, both its next and previous links point to itself.
	//!
	//! @param item The item to insert into the list. The item must not already be on the list.
	void List::add(List::Node *item) {
		assert(item->m_next == nullptr && item->m_prev == nullptr);

		// Handle an empty list.
		if (!m_head) {
			m_head = item;
			item->m_next = item;
			item->m_prev = item;
		}
		// Insert at end of list if there is no sorting predicate, or if the item sorts after the
		// last item in the list.
		else if (!m_predicate || !m_predicate(item, m_head->m_prev)) {
			item->insertBefore(m_head);
		}
		// Otherwise, search for the sorted position in the list for the item to be inserted.
		else {
			// Insert sorted by priority.
			List::Node *node = m_head;

			do {
				if (m_predicate(item, node)) {
					item->insertBefore(node);

					if (node == m_head) { m_head = item; }

					break;
				}

				node = node->m_next;
			} while (node != m_head);
		}

		if constexpr (Ar::config::ENABLE_LIST_CHECKS) {
			check();
		}
	}

	//! If the specified item is not on the list, nothing happens. Items are compared only by pointer
	//! value, *not* by using the predicate function.
	//!
	//! @param item The item to remove from the list.
	void List::remove(List::Node *item) {
		// the list must not be empty
		assert(m_head != nullptr);
		assert(item->m_next);
		assert(item->m_prev);
		if constexpr (Ar::config::ENABLE_LIST_CHECKS) {
			assert(contains(item));
		}

		// Adjust other nodes' links.
		item->m_prev->m_next = item->m_next;
		item->m_next->m_prev = item->m_prev;

		// Special case for removing the list head.
		if (m_head == item) {
			// Handle a single item list by clearing the list head.
			if (item->m_next == m_head) {
				m_head = nullptr;
			}
			// Otherwise just update the list head to the second list element.
			else {
				m_head = item->m_next;
			}
		}

		// Clear links.
		item->m_next = nullptr;
		item->m_prev = nullptr;

		if constexpr (Ar::config::ENABLE_LIST_CHECKS) {
			check();
		}
	}

	void List::check() {
		const uint32_t kNumNodes = 20;
		List::Node *nodes[kNumNodes] = {0};
		uint32_t count = 0;
		uint32_t i;

		// Handle empty list.
		if (isEmpty()) { return; }

		// Build array of all nodes in the list.
		List::Node *node = m_head;
		bool loop = true;
		while (loop) {
			// Save this node in the list.
			nodes[count] = node;
			++count;
			if (count == kNumNodes - 1) {
				// More nodes than we have room for!
				Ar::Port::halt();
			}

			node = node->m_next;

			// Compare the next ptr against every node we've seen so far. If we find
			// a match, exit the loop.
			for (i = 0; i < count; ++i) {
				if (node == nodes[i]) {
					loop = false;
					break;
				}
			}
		}

		// Scan the nodes array and verify all links.
		for (i = 0; i < count; ++i) {
			uint32_t prev = (i == 0) ? (count - 1) : (i - 1);
			uint32_t next = (i == count - 1) ? 0 : (i + 1);

			node = nodes[i];
			if (node->m_next != nodes[next]) { Ar::Port::halt(); }
			if (node->m_prev != nodes[prev]) { Ar::Port::halt(); }
		}
	}
}