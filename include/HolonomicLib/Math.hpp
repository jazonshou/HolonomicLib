#pragma once
#define _USE_MATH_DEFINES
#include "HolonomicWheelSpeeds.hpp"
#include <utility>
#include "okapi/api/units/QAngle.hpp"
#include "math.h"


namespace HolonomicLib {

namespace Math {

    std::pair<double, double> rotateVector(double x, double y, okapi::QAngle angle);

}


namespace HolonomicMath {
    HolonomicWheelSpeeds move(double ySpeed, double xSpeed, double zRotation, okapi::QAngle imuAngle);
}
}