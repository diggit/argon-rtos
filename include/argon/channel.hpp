#if !defined(_AR_CHANNEL_H_)
	#define _AR_CHANNEL_H_

	#include "argon/list.hpp"
	#include "argon/common.hpp"
	#include <cstdint>
	#include <optional>

namespace Ar {

	/*!
	 * @brief Channel.
	 *
	 * @ingroup ar_chan
	 */
	class Channel {
		Name m_name {}; //!< Name of the channel.
		std::uint32_t m_width; //!< Size in bytes of the channel's data.
		List m_blockedSenders {}; //!< List of blocked sender threads.
		List m_blockedReceivers {}; //!< List of blocked receiver threads.
	#if AR_GLOBAL_OBJECT_LISTS
		List::Node m_createdNode; //!< Node on the created channels list.
	#endif // AR_GLOBAL_OBJECT_LISTS
	  public:
		//! @brief Default constructor.
		Channel() = delete;

		//! @brief Constructor.
		Channel(const char *name, std::size_t width = 0);

		//! @brief Destructor.
		~Channel();

		//! @brief Send to channel.
		Ar::Status receive(void *value, std::optional<Duration> timeout = std::nullopt) {
			return send_receive(false, m_blockedReceivers, m_blockedSenders, value, timeout);
		}

		//! @brief Receive from channel.
		Ar::Status send(const void *value, std::optional<Duration> timeout = std::nullopt) {
			return send_receive(true, m_blockedSenders, m_blockedReceivers, const_cast<void *>(value), timeout);
		}

		const char *getName() { return m_name.data(); }

	  private:
		//! @brief Disable copy constructor.
		Channel(const Channel &other) = delete;

		//! @brief Disable assignment operator.
		Channel &operator=(const Channel &other) = delete;

		Status block(List &myDirList, void *value, std::optional<Duration> timeout = std::nullopt);

		Status send_receive(bool isSending, List &myDirList, List &otherDirList, void *value, std::optional<Duration> timeout = std::nullopt);

		Status send_receive_internal(bool isSending, List &myDirList, List &otherDirList, void *value, std::optional<Duration> timeout = std::nullopt);

		static void deferred_send(void *object, void *object2);
	};

	/*!
	 * @brief Typed channel.
	 *
	 * @ingroup ar_chan
	 */
	template<typename T>
	class TypedChannel : public Channel {
	  public:
		//! @brief Default constructor.
		TypedChannel() = delete;

		//! @brief Constructor.
		TypedChannel(const char *name) : Channel(name, sizeof(T)) {}

		//! @brief Send to channel.
		Ar::Status send(const T &value, std::optional<Duration> timeout = std::nullopt) { return Channel::send(&value, timeout); }

		//! @brief Receive from channel.
		T receive(std::optional<Duration> timeout = std::nullopt) {
			T temp;
			Channel::receive(&temp, timeout);
			return temp;
		}

		//! @brief Receive from channel.
		Ar::Status receive(T &value, std::optional<Duration> timeout = std::nullopt) { return Channel::receive(&value, timeout); }

		//! @brief Receive from channel.
		friend T &operator<<=(T &lhs, TypedChannel<T> &rhs) {
			lhs = rhs.receive();
			return lhs;
		}

		//! @brief Send to channel.
		friend T &operator>>(T &lhs, TypedChannel<T> &rhs) {
			rhs.send(lhs);
			return lhs;
		}

	  private:
		//! @brief Disable copy constructor.
		TypedChannel(const TypedChannel<T> &other) = delete;

		//! @brief Disable assignment operator.
		TypedChannel &operator=(const TypedChannel<T> &other) = delete;
	};
} // namespace Ar
#endif