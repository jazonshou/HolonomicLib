#include "Math.hpp"
#include <algorithm>

namespace HolonomicLib {

namespace Math {

okapi::QAngle rescale180(okapi::QAngle angle) {
    return rescale180(angle.convert(okapi::degree)) * okapi::degree;
}

double rescale180(double angle) {
    return angle - 360.0 * std::floor((angle + 180.0) * (1.0 / 360.0));
}

} // namespace Math

} // namespace HolonomicLib
