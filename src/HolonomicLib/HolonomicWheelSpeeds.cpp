#pragma once
#include "HolonomicWheelSpeeds.hpp"

namespace HolonomicLib {

HolonomicWheelSpeeds::HolonomicWheelSpeeds(double frontleft, double frontRight, double backLeft, double backRight) {
    frontLeft = frontleft;
    frontRight = frontRight;
    backLeft = backLeft;
    backRight = backRight;
}

}