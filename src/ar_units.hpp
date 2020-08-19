#ifndef _AR_UNITS_H_
#define _AR_UNITS_H_

#include <cstdint>
#include <type_traits>

namespace Ar::units {

	using literalFloat = long double;
	using literalInt = unsigned long long int;

	// this template brilliance comes from VoMi
	template<typename... Args>
	struct static_list {};

	template<typename... Args>
	using list_of = static_list<Args...>;

	template<typename... Args>
	struct list_contains_impl;

	template<template<typename...> class List, typename Searched, typename Arg>
	struct list_contains_impl<List<Arg>, Searched> {
		constexpr static bool value = std::is_same_v<Arg, Searched>;
	};

	template<template<typename...> class List, typename Searched, typename Arg, typename... Args>
	struct list_contains_impl<List<Arg, Args...>, Searched> {
		constexpr static bool value = std::is_same_v<Arg, Searched> || list_contains_impl<List<Args...>, Searched>::value;
	};

	template<typename List, typename Searched>
	struct list_contains {
		static constexpr bool value = list_contains_impl<List, Searched>::value;
	};

	template<typename List, typename Searched>
	constexpr inline bool list_contains_v = list_contains<List, Searched>::value;

	namespace OperatorTypes {
		struct Comparison {};
		struct AddSub {};
		struct MulDiv {};
	} // namespace OperatorTypes

	// rescales source raw value to target raw value based on oneBu ratio
	template<class Source, class Target>
	constexpr typename Target::baseType rawConvert(typename Source::baseType sourceValue) {
		constexpr typename Target::baseType targetBu = Target::oneBU;
		constexpr typename Source::baseType sourceBu = Source::oneBU;
		if constexpr (targetBu >= sourceBu) {
			return sourceValue * (targetBu / sourceBu);
		} else {
			return sourceValue / (sourceBu / targetBu);
		}
	}

	template<class Derived>
	struct CommonUnitOperators {
		// relation operators

		constexpr bool operator<(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return me.value < r.value;
		}

		constexpr bool operator>(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return me.value > r.value;
		}

		constexpr bool operator==(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return me.value == r.value;
		}

		constexpr bool operator!=(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return me.value != r.value;
		}

		constexpr bool operator<=(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return me.value <= r.value;
		}

		constexpr bool operator>=(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return me.value >= r.value;
		}

		// ratio
		constexpr float operator/(const Derived &r) const {
			auto &me = static_cast<const Derived &>(*this);
			return static_cast<float>(me.value) / r.value;
		}

		// arithmetic operators
		constexpr Derived operator/(float r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::MulDiv>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(me.value / r);
		}

		constexpr Derived operator*(float r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::MulDiv>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(me.value * r);
		}

		constexpr Derived operator/(int r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::MulDiv>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(static_cast<float>(me.value) / static_cast<float>(r));
		}

		constexpr Derived operator*(int r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::MulDiv>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(static_cast<float>(me.value) * static_cast<float>(r));
		}

		// arithmetic operators of same type
		constexpr Derived operator+(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::AddSub>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(me.value + r.value);
		}

		constexpr Derived operator-(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::AddSub>, "Your type must add this requirement.");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(me.value - r.value);
		}

		// inc dec assign operators
		constexpr Derived &operator-=(const Derived &r) {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::AddSub>, "Your type must add this requirement.");
			auto &me = static_cast<Derived &>(*this);
			me.value -= r.value;
			return me;
		};

		constexpr Derived &operator+=(const Derived &r) {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::AddSub>, "Your type must add this requirement.");
			auto &me = static_cast<Derived &>(*this);
			me.value += r.value;
			return me;
		};

		constexpr Derived operator-() const {
			auto &me = static_cast<const Derived &>(*this);
			static_assert(std::is_signed_v<decltype(me.value)>, "baseType type must be signed to apply unary minus operator.");
			return {Derived(static_cast<decltype(me.value)>(-me.value))};
		}

		constexpr CommonUnitOperators() = default;
		friend Derived;
	};

	template<typename Derived, typename baseType_, baseType_ oneBU_>
	struct CommonUnitBase {
		using baseType = baseType_;
		static constexpr baseType oneBU = oneBU_; // should be 10^n
		static constexpr baseType baseTypeBase = 10;
		static_assert(oneBU % baseTypeBase == 0, "oneBU must be 10^n");

		[[nodiscard]] constexpr baseType getRaw() const { return value; };

	  protected:
		baseType value;
		explicit constexpr CommonUnitBase(baseType raw) : value(raw) {};

		static constexpr std::uint32_t _1k = 1'000;
		static constexpr std::uint32_t _1M = 1'000'000UL;
		static constexpr std::uint32_t _1B = 1'000'000'000UL;

		static constexpr float _1kf = 1e3;
		static constexpr float _1Mf = 1e6;
		static constexpr float _1Bf = 1e9;
	};

	class Duration : public CommonUnitBase<Duration, std::uint64_t, 1'000'000UL>, public CommonUnitOperators<Duration> {
		using flags = list_of<OperatorTypes::Comparison, OperatorTypes::AddSub, OperatorTypes::MulDiv>;
		using CommonUnitBase::CommonUnitBase; // ctor
		friend CommonUnitOperators;

	  public:
		// constexpr baseType getRaw() const {return value;};
		// use exponential
		[[nodiscard]] constexpr float in_s_f() const { return static_cast<float>(value) / oneBU; };
		[[nodiscard]] constexpr float in_ms_f() const { return static_cast<float>(value) / (oneBU / _1k); }
		[[nodiscard]] constexpr float in_us_f() const { return static_cast<float>(value) / (oneBU / _1M); }

		[[nodiscard]] constexpr std::uint32_t in_s() const { return value / oneBU; };
		[[nodiscard]] constexpr std::uint64_t in_ms() const { return value / (oneBU / _1k); }
		[[nodiscard]] constexpr std::uint64_t in_us() const { return value / (oneBU / _1M); }

		[[nodiscard]] static constexpr Duration zero() { return Duration(0); };
		template<typename T>
		[[nodiscard]] static constexpr Duration from_s(T value) {
			static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value, "argument must be integer or floating type");
			return Duration(value * oneBU);
		}

		template<typename T>
		[[nodiscard]] static constexpr Duration from_ms(T value) {
			static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value, "argument must be integer or floating type");
			return Duration(value * (oneBU / _1k));
		}

		template<typename T>
		[[nodiscard]] static constexpr Duration from_us(T value) {
			static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value, "argument must be integer or floating type");
			return Duration(value * (oneBU / _1M));
		}

		[[nodiscard]] static constexpr Duration from_raw(baseType raw) { return Duration(raw); };

		operator bool() const { return value; }
	};

	// 1 us resolution
	class Time : public CommonUnitBase<Time, std::uint64_t, 1'000'000UL>, public CommonUnitOperators<Time> {
		using flags = list_of<OperatorTypes::Comparison>;
		using CommonUnitBase::CommonUnitBase; // ctor
		friend CommonUnitOperators;

	  public:
		// constexpr baseType getRaw() const {return value;};
		// use exponential
		[[nodiscard]] constexpr float in_s_f() const { return static_cast<float>(value) / oneBU; };
		[[nodiscard]] constexpr float in_ms_f() const { return static_cast<float>(value) / (oneBU / _1k); }
		[[nodiscard]] constexpr float in_us_f() const { return static_cast<float>(value) / (oneBU / _1M); }

		[[nodiscard]] constexpr std::uint32_t in_s() const { return value / oneBU; };
		[[nodiscard]] constexpr std::uint64_t in_ms() const { return value / (oneBU / _1k); }
		[[nodiscard]] constexpr std::uint64_t in_us() const { return value / (oneBU / _1M); }

		[[nodiscard]] constexpr Time operator+(const Duration &r) const {
			return Time(value + rawConvert<Duration, Time>(r.getRaw())); // optimal only for ms as base
		}

		[[nodiscard]] constexpr Time operator-(const Duration &r) const {
			return Time(value - rawConvert<Duration, Time>(r.getRaw())); // optimal only for ms as base
		}

		// WARNING: returned Duration is alway positive (abs value of difference)!
		[[nodiscard]] constexpr Duration operator-(const Time &r) const {
			if (value >= r.value) { return Duration::from_raw(rawConvert<Time, Duration>(value - r.value)); }
			return Duration::from_raw(rawConvert<Time, Duration>(r.value - value));
		}

		[[nodiscard]] constexpr Time &operator-=(const Duration &r) {
			value -= rawConvert<Duration, Time>(r.getRaw());
			return *this;
		};

		[[nodiscard]] constexpr Time &operator+=(const Duration &r) {
			value += rawConvert<Duration, Time>(r.getRaw());
			return *this;
		};

		[[nodiscard]] bool elapsed(const Duration &duration) { return (now() - *this) > duration; }

		[[nodiscard]] static Time now();

		[[nodiscard]] static constexpr Time zero() {return Time(0);} //normally never used, meant for disabled SystemLoad etc.
	};

} // namespace Ar::units

namespace Ar {
	using Time = Ar::units::Time;
	using Duration = Ar::units::Duration;
} // namespace Ar

[[nodiscard]] constexpr Ar::units::Duration operator""_s(Ar::units::literalFloat x) { return Ar::units::Duration::from_s(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Duration operator""_s(Ar::units::literalInt x) { return Ar::units::Duration::from_s(static_cast<Ar::units::Duration::baseType>(x)); }
[[nodiscard]] constexpr Ar::units::Duration operator""_ms(Ar::units::literalFloat x) { return Ar::units::Duration::from_ms(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Duration operator""_ms(Ar::units::literalInt x) { return Ar::units::Duration::from_ms(static_cast<Ar::units::Duration::baseType>(x)); }
[[nodiscard]] constexpr Ar::units::Duration operator""_us(Ar::units::literalFloat x) { return Ar::units::Duration::from_us(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Duration operator""_us(Ar::units::literalInt x) { return Ar::units::Duration::from_us(static_cast<Ar::units::Duration::baseType>(x)); }

#endif