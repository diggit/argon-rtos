#ifndef _UNITS_HEADER_INCLUDED
#define _UNITS_HEADER_INCLUDED

#include <cstdint>
#include <type_traits>
#include <chrono>

namespace Ar::units {

	template<typename T, typename P>
	using typeAndRatio = std::chrono::duration<T, P>;

	template<typename T, typename U, typename V>
	constexpr T typeAndRatio_cast(std::chrono::duration<U, V> duration_) {
		return std::chrono::duration_cast<T>(duration_);
	}

	// template<typename T>
	// using try_make_unsigned = std::conditional_t<std::is_integral_v<T>, std::make_unsigned_t<std::conditional_t<std::is_integral_v<T>, T, int>>, T>; // ugly
	// but works, improvements welcome!

	template<typename T>
	using try_make_unsigned_alt = typename std::conditional<std::is_integral<T>::value, std::make_unsigned<T>, std::type_identity<T>>::type;

	template<typename Tvalue>
	constexpr typename try_make_unsigned_alt<Tvalue>::type tabs(Tvalue value) {
		static_assert(std::is_scalar_v<Tvalue>, "value must be scalar");
		if constexpr (std::is_signed_v<Tvalue>) {
			if (value < 0) { return -value; }
		}
		return value;
	}

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
		struct Add {};
		struct Sub {};
		struct Mul {};
		struct Div {};
	} // namespace OperatorTypes

	// rescales source raw value to target raw value based on oneBu ratio
	// template<class Source, class Target>
	// constexpr typename Target::BaseType rawConvert(typename Source::BaseType sourceValue) {
	// 	constexpr typename Target::BaseType targetBu = Target::oneBU;
	// 	constexpr typename Source::BaseType sourceBu = Source::oneBU;
	// 	if constexpr (targetBu >= sourceBu) {
	// 		return sourceValue * (targetBu / sourceBu);
	// 	} else {
	// 		return sourceValue / (sourceBu / targetBu);
	// 	}
	// }

	template<class Derived>
	struct CommonUnitOperators {
		// relation operators

		constexpr bool operator<(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Comparison, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return me.value < r.value;
		}

		constexpr bool operator>(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Comparison, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return me.value > r.value;
		}

		constexpr bool operator==(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Comparison, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return me.value == r.value;
		}

		constexpr bool operator!=(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Comparison, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return me.value != r.value;
		}

		constexpr bool operator<=(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Comparison, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return me.value <= r.value;
		}

		constexpr bool operator>=(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Comparison>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Comparison, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return me.value >= r.value;
		}

		// ratio
		constexpr float operator/(const Derived &r) const {
			auto &me = static_cast<const Derived &>(*this);
			return static_cast<float>(me.value) / r.value;
		}

		// arithmetic operators

		constexpr Derived operator*(float r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Mul>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Mul, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(me.value * r);
		}

		constexpr Derived operator*(int r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Mul>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Mul, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(static_cast<float>(me.value) * static_cast<float>(r));
		}

		constexpr Derived operator/(float r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Div>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Div, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(me.value / r);
		}

		constexpr Derived operator/(int r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Div>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Div, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(static_cast<float>(me.value) / static_cast<float>(r));
		}

		// arithmetic operators of same type
		constexpr Derived operator+(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Add>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Add, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(me.value + r.value);
		}

		constexpr Derived &operator+=(const Derived &r) {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Add>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Add, ...>;).");
			auto &me = static_cast<Derived &>(*this);
			me.value += r.value;
			return me;
		};

		constexpr Derived operator-(const Derived &r) const {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Sub>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Sub, ...>;).");
			auto &me = static_cast<const Derived &>(*this);
			return Derived(me.value - r.value);
		}

		// inc dec assign operators
		constexpr Derived &operator-=(const Derived &r) {
			static_assert(list_contains_v<typename Derived::flags, OperatorTypes::Sub>,
				"Your type must enable this operation (using flags = list_of<OperatorTypes::Sub, ...>;).");
			auto &me = static_cast<Derived &>(*this);
			me.value -= r.value;
			return me;
		};

		constexpr Derived operator-() const {
			auto &me = static_cast<const Derived &>(*this);
			static_assert(std::is_signed_v<decltype(me.value)>, "BaseType type must be signed to apply unary minus operator.");
			return {Derived(static_cast<decltype(me.value)>(-me.value))};
		}

		constexpr CommonUnitOperators() = default;
		friend Derived;
	};

	template<typename Derived, typename BaseType_, class BaseResolution_ = std::ratio<1, 1>, bool allowTruncation_ = false>
	struct CommonUnitBase {
		static constexpr auto allowTruncation = allowTruncation_;
		using BaseType = BaseType_;
		using BaseResolution = BaseResolution_;
		static_assert(!std::is_floating_point_v<BaseType> || std::is_same_v<BaseResolution, typename std::ratio<1, 1>>,
			"floating point should have 1:1 resolution, because you know.... floating point...");
		using baseTypeAndRatio = typeAndRatio<BaseType, BaseResolution>;
		// static_assert(resolution::num == 1, "numerator must be == 1");

		[[nodiscard]] constexpr BaseType getRaw() const { return value; };

	  protected:
		BaseType value;
		explicit constexpr CommonUnitBase(BaseType raw) : value(raw) {};

		template<bool allowTruncationOveride, typename targetResolution, typename targetType, typename sourceResolution, typename sourceType>
		[[nodiscard]] static constexpr targetType resolutionConversion(const sourceType &value_) {
			// using sourceType = std::remove_const_t<std::remove_reference_t<decltype(value_)>>;
			static_assert(std::is_floating_point_v<sourceType> || std::is_integral_v<sourceType>, "sourceType must be float or integral type");

			if constexpr (std::is_signed_v<sourceType> && !std::is_signed_v<targetType>) {
				// from signed to unsigned
				if (value_ < 0) {
					// TODO: assert?
					return 0; // return lowest possible value preventing overflow
				}
			}

			if constexpr ((std::is_floating_point_v<sourceType> && !std::is_floating_point_v<targetType>) || allowTruncation
						  || allowTruncationOveride) { // typeAndRatio cosiders float->int always truncating
				return typeAndRatio_cast<typeAndRatio<targetType, targetResolution>>(typeAndRatio<sourceType, sourceResolution>(value_))
					.count(); // explicit duration_cast disables truncation checks
			} else {
				return typeAndRatio<targetType, targetResolution>(typeAndRatio<sourceType, sourceResolution>(value_)).count();
			}
		}

	  public:
		template<typename sourceResolution = std::ratio<1, 1>, bool allowTruncationOveride = false>
		[[nodiscard]] static constexpr Derived from(auto value_) {
			return Derived(resolutionConversion<allowTruncationOveride, BaseResolution, BaseType, sourceResolution, decltype(value_)>(value_));
		}

		template<typename sourceType, typename sourceRatio>
		[[nodiscard]] static constexpr Derived fromTypeAndRatio(typeAndRatio<sourceType, sourceRatio> source_) {
			return Derived(resolutionConversion<true, BaseResolution, BaseType, sourceRatio>(source_.count()));
		}

		[[nodiscard]] static constexpr Derived fromRaw(BaseType value_) { return Derived(value_); }

		template<typename outputResolution, typename outputType = BaseType>
		[[nodiscard]] constexpr outputType in() const {
			return resolutionConversion<true, outputResolution, outputType, BaseResolution>(value);
		}

		template<typename outputType = BaseType>
		[[nodiscard]] constexpr outputType inBasicUnit() const {
			return resolutionConversion<true, std::ratio<1, 1>, outputType, BaseResolution>(value);
		}

		[[nodiscard]] constexpr typeAndRatio<BaseType, BaseResolution> asTypeAndRatio() const { return typeAndRatio<BaseType, BaseResolution>(value); }

		using UnsignedBaseType = typename try_make_unsigned_alt<BaseType>::type;
		[[nodiscard]] constexpr typeAndRatio<UnsignedBaseType, BaseResolution> asTypeAndRatioAbs() const {
			return typeAndRatio<UnsignedBaseType, BaseResolution>(tabs(value));
		}

		operator bool() const { return value; }
	};

	// ===================== Unit definitions =====================
	class Duration;
	class Time;

	// ------------------------- Duration -------------------------

	class Duration : public CommonUnitBase<Duration, std::uint64_t, std::micro>, public CommonUnitOperators<Duration> {
		using flags = list_of<OperatorTypes::Comparison, OperatorTypes::Add, OperatorTypes::Sub, OperatorTypes::Mul, OperatorTypes::Div>;
		using CommonUnitBase::CommonUnitBase; // ctor
		friend CommonUnitOperators;
		friend Time;

	  public:
		constexpr operator std::chrono::duration<BaseType, BaseResolution>() { return std::chrono::duration<BaseType, BaseResolution>(value); }

		static constexpr Duration zero() { return Duration(0); }

		auto chrono() const {
			return std::chrono::duration<BaseType, BaseResolution>(value);
		}

		struct AllowTruncation {};
		static constexpr AllowTruncation allowTruncation {};

		// truncation non-permissive
		template<typename SourceType, typename SourceResolution>
		constexpr Duration(std::chrono::duration<SourceType, SourceResolution> duration) :
			Duration(resolutionConversion<false, BaseResolution, BaseType, SourceResolution>(duration.count())) {}

		// truncation permissive
		template<typename SourceType, typename SourceResolution>
		constexpr Duration(std::chrono::duration<SourceType, SourceResolution> duration, AllowTruncation tag) :
			Duration(resolutionConversion<true, BaseResolution, BaseType, SourceResolution>(duration.count())) {}
	};

	// --------------------------- Time ---------------------------

	class Time : public CommonUnitBase<Time, std::uint64_t, std::micro>, public CommonUnitOperators<Time> {
		using flags = list_of<OperatorTypes::Comparison>;
		using CommonUnitBase::CommonUnitBase; // ctor
		friend CommonUnitOperators;

	  public:
		[[nodiscard]] constexpr Time operator+(const Duration &r) const {
			return Time(value + resolutionConversion<true, BaseResolution, BaseType, Duration::BaseResolution>(r.getRaw()));
		}

		[[nodiscard]] constexpr Time operator-(const Duration &r) const {
			return Time(value - resolutionConversion<true, BaseResolution, BaseType, Duration::BaseResolution>(r.getRaw()));
		}

		// // WARNING: returned Duration is alway positive (abs value of difference)!
		[[nodiscard]] constexpr Duration operator-(const Time &r) const {
			BaseType v;

			if constexpr (std::is_signed_v<Duration::BaseType>) {
				v = value - r.value;
			} else {
				if (value >= r.value) {
					v = value - r.value;
				} else {
					v = r.value - value;
				}
			}
			return Duration::fromRaw(resolutionConversion<true, Duration::BaseResolution, Duration::BaseType, BaseResolution>(v));
		}

		[[nodiscard]] constexpr Time &operator-=(const Duration &r) {
			value -= resolutionConversion<true, BaseResolution, BaseType, Duration::BaseResolution>(r.getRaw());
			return *this;
		};

		[[nodiscard]] constexpr Time &operator+=(const Duration &r) {
			value += resolutionConversion<true, BaseResolution, BaseType, Duration::BaseResolution>(r.getRaw());
			return *this;
		};

		[[nodiscard]] bool elapsed(const Duration &duration) { return (now() - *this) > duration; }

		[[nodiscard]] static Time now();

		[[nodiscard]] static constexpr Time zero() { return Time(0); } // normally never used, meant for disabled SystemLoad etc.
	};

	class Voltage : public CommonUnitBase<Voltage, float, std::ratio<1, 1>, true>, public CommonUnitOperators<Voltage> {
		using flags = list_of<OperatorTypes::Comparison, OperatorTypes::Add, OperatorTypes::Sub, OperatorTypes::Mul, OperatorTypes::Div>;
		using CommonUnitBase::CommonUnitBase; // ctor
		friend CommonUnitOperators;
	};

	class Current : public CommonUnitBase<Current, float, std::ratio<1, 1>, true>, public CommonUnitOperators<Current> {
		using flags = list_of<OperatorTypes::Comparison, OperatorTypes::Add, OperatorTypes::Sub, OperatorTypes::Mul, OperatorTypes::Div>;
		using CommonUnitBase::CommonUnitBase; // ctor
		friend CommonUnitOperators;
	};

	class Power : public CommonUnitBase<Power, float, std::ratio<1, 1>, true>, public CommonUnitOperators<Power> {
		using flags = list_of<OperatorTypes::Comparison, OperatorTypes::Add, OperatorTypes::Sub, OperatorTypes::Mul, OperatorTypes::Div>;
		using CommonUnitBase::CommonUnitBase; // ctor
		friend CommonUnitOperators;
	};

} // namespace units

// ---------------------- User literals ---------------------------

[[nodiscard]] constexpr Ar::units::Duration operator""_s(Ar::units::literalFloat x) { return Ar::units::Duration::from<std::ratio<1, 1>>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Duration operator""_s(Ar::units::literalInt x) {
	return Ar::units::Duration::from<std::ratio<1, 1>>(static_cast<Ar::units::Duration::BaseType>(x));
}
[[nodiscard]] constexpr Ar::units::Duration operator""_ms(Ar::units::literalFloat x) { return Ar::units::Duration::from<std::milli>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Duration operator""_ms(Ar::units::literalInt x) {
	return Ar::units::Duration::from<std::milli>(static_cast<Ar::units::Duration::BaseType>(x));
}
[[nodiscard]] constexpr Ar::units::Duration operator""_us(Ar::units::literalFloat x) { return Ar::units::Duration::from<std::micro>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Duration operator""_us(Ar::units::literalInt x) {
	return Ar::units::Duration::from<std::micro>(static_cast<Ar::units::Duration::BaseType>(x));
}

[[nodiscard]] constexpr Ar::units::Voltage operator""_V(Ar::units::literalFloat x) { return Ar::units::Voltage::from<std::ratio<1, 1>>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Voltage operator""_V(Ar::units::literalInt x) {
	return Ar::units::Voltage::from<std::ratio<1, 1>>(static_cast<Ar::units::Voltage::BaseType>(x));
}
[[nodiscard]] constexpr Ar::units::Voltage operator""_mV(Ar::units::literalFloat x) { return Ar::units::Voltage::from<std::milli>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Voltage operator""_mV(Ar::units::literalInt x) { return Ar::units::Voltage::from<std::milli>(static_cast<Ar::units::Voltage::BaseType>(x)); }
[[nodiscard]] constexpr Ar::units::Voltage operator""_uV(Ar::units::literalFloat x) { return Ar::units::Voltage::from<std::micro>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Voltage operator""_uV(Ar::units::literalInt x) {
	return Ar::units::Voltage::from<std::micro>(static_cast<Ar::units::Duration::BaseType>(x));
}

[[nodiscard]] constexpr Ar::units::Current operator""_A(Ar::units::literalFloat x) { return Ar::units::Current::from<std::ratio<1, 1>>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Current operator""_A(Ar::units::literalInt x) {
	return Ar::units::Current::from<std::ratio<1, 1>>(static_cast<Ar::units::Current::BaseType>(x));
}
[[nodiscard]] constexpr Ar::units::Current operator""_mA(Ar::units::literalFloat x) { return Ar::units::Current::from<std::milli>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Current operator""_mA(Ar::units::literalInt x) { return Ar::units::Current::from<std::milli>(static_cast<Ar::units::Current::BaseType>(x)); }
[[nodiscard]] constexpr Ar::units::Current operator""_uA(Ar::units::literalFloat x) { return Ar::units::Current::from<std::micro>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Current operator""_uA(Ar::units::literalInt x) {
	return Ar::units::Current::from<std::micro>(static_cast<Ar::units::Duration::BaseType>(x));
}

[[nodiscard]] constexpr Ar::units::Power operator""_kW(Ar::units::literalFloat x) { return Ar::units::Power::from<std::kilo>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Power operator""_kW(Ar::units::literalInt x) { return Ar::units::Power::from<std::kilo>(static_cast<Ar::units::Power::BaseType>(x)); }
[[nodiscard]] constexpr Ar::units::Power operator""_W(Ar::units::literalFloat x) { return Ar::units::Power::from<std::ratio<1, 1>>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Power operator""_W(Ar::units::literalInt x) { return Ar::units::Power::from<std::ratio<1, 1>>(static_cast<Ar::units::Power::BaseType>(x)); }
[[nodiscard]] constexpr Ar::units::Power operator""_mW(Ar::units::literalFloat x) { return Ar::units::Power::from<std::milli>(static_cast<float>(x)); }
[[nodiscard]] constexpr Ar::units::Power operator""_mW(Ar::units::literalInt x) { return Ar::units::Power::from<std::milli>(static_cast<Ar::units::Power::BaseType>(x)); }

// -------------------------- special operators ----------------------------

template<typename Lhs, typename Rhs>
constexpr auto tarMul(const Lhs &lhs, const Rhs &rhs) {
	static_assert(std::is_floating_point_v<typename Lhs::rep>, "Lhs base type must be floating point");
	static_assert(std::is_floating_point_v<typename Rhs::rep>, "Rhs base type must be floating point");
	static_assert(std::is_same_v<typename Lhs::period, typename std::ratio<1, 1>>, "Lhs ration<1,1> is expected, why would you use anything else with float?");
	static_assert(std::is_same_v<typename Rhs::period, typename std::ratio<1, 1>>, "Rhs ration<1,1> is expected, why would you use anything else with float?");
	return Lhs(lhs.count() * rhs.count());
}

template<typename Lhs, typename Rhs>
constexpr auto tarDiv(const Lhs &lhs, const Rhs &rhs) {
	static_assert(std::is_floating_point_v<typename Lhs::rep>, "Lhs base type must be floating point");
	static_assert(std::is_floating_point_v<typename Rhs::rep>, "Rhs base type must be floating point");
	static_assert(std::is_same_v<typename Lhs::period, typename std::ratio<1, 1>>, "Lhs ration<1,1> is expected, why would you use anything else with float?");
	static_assert(std::is_same_v<typename Rhs::period, typename std::ratio<1, 1>>, "Rhs ration<1,1> is expected, why would you use anything else with float?");
	return Lhs(lhs.count() / rhs.count());
}

constexpr Ar::units::Power operator*(const Ar::units::Voltage &v, const Ar::units::Current &c) {
	return Ar::units::Power::fromTypeAndRatio(tarMul(v.asTypeAndRatioAbs(), c.asTypeAndRatioAbs()));
}

constexpr Ar::units::Power operator*(const Ar::units::Current &c, const Ar::units::Voltage &v) {
	return Ar::units::Power::fromTypeAndRatio(tarMul(c.asTypeAndRatioAbs(), v.asTypeAndRatioAbs()));
}

constexpr Ar::units::Current operator/(const Ar::units::Power &p, const Ar::units::Voltage &v) {
	return Ar::units::Current::fromTypeAndRatio(tarDiv(p.asTypeAndRatioAbs(), v.asTypeAndRatioAbs()));
}

constexpr Ar::units::Voltage operator/(const Ar::units::Power &p, const Ar::units::Current &c) {
	return Ar::units::Voltage::fromTypeAndRatio(tarDiv(p.asTypeAndRatioAbs(), c.asTypeAndRatioAbs()));
}

namespace Ar {
	using Time = Ar::units::Time;
	using Duration = Ar::units::Duration;
} // namespace Ar

#endif