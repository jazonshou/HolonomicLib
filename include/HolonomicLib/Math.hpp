#pragma once
#define _USE_MATH_DEFINES
#include "HolonomicWheelSpeeds.hpp"
#include <utility>
#include "okapi/api/units/QAngle.hpp"
#include "math.h"


namespace HolonomicLib {

namespace Math {

okapi::QAngle rescale180(okapi::QAngle angle);
double rescale180(double angle);

}


// namespace HolonomicMath {
//     HolonomicWheelSpeeds move(double ySpeed, double xSpeed, double zRotation, okapi::QAngle imuAngle);
// }
}