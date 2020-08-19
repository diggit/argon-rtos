#include "ar_units.hpp"
#include "ar_port.hpp"

using namespace Ar::units;

Time Time::now() {
	return Time(Ar::Port::get_time_absolute_us());
}