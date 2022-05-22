#include "HolonomicWheelSpeeds.hpp"

namespace HolonomicLib {

HolonomicWheelSpeeds::HolonomicWheelSpeeds(double frontleft, double frontRight, double backLeft, double backRight) {
    this->frontLeft = frontleft;
    this->frontRight = frontRight;
    this->backLeft = backLeft;
    this->backRight = backRight;
}

}