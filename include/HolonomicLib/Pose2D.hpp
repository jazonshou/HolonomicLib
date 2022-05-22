#pragma once
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"

namespace HolonomicLib {

struct Pose2D
{
    okapi::QLength x, y;
    okapi::QAngle theta;
    Pose2D() = default;
    Pose2D(okapi::QLength ix, okapi::QLength iy, okapi::QAngle itheta);
};

}