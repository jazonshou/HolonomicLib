#pragma once

namespace HolonomicLib {

struct HolonomicWheelSpeeds
{
    double frontLeft, frontRight, backLeft, backRight;
    HolonomicWheelSpeeds(double frontleft, double frontRight, double backLeft, double backRight);
};


}