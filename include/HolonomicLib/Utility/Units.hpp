#pragma once
#include "okapi/api/units/QAcceleration.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QTime.hpp"


namespace okapi {

constexpr QSpeed ftps = foot / second;
constexpr QSpeed inps = inch / second;
constexpr QAcceleration ftps2 = foot / (second * second);

constexpr QSpeed operator"" _ftps(long double x) {
    return static_cast<double>(x) * ftps;
}
constexpr QSpeed operator"" _ftps(unsigned long long int x) {
    return static_cast<double>(x) * ftps;
}
constexpr QSpeed operator"" _inps(long double x) {
    return static_cast<double>(x) * inps;
}
constexpr QSpeed operator"" _inps(unsigned long long int x) {
    return static_cast<double>(x) * inps;
}
constexpr QAcceleration operator"" _ftps2(long double x) {
    return static_cast<double>(x) * ftps2;
}
constexpr QAcceleration operator"" _ftps2(unsigned long long int x) {
    return static_cast<double>(x) * ftps2;
}

} // namespace okapi