#include "Math.hpp"
#include <algorithm>

namespace HolonomicLib {

namespace HolonomicMath {

    HolonomicWheelSpeeds move(double ySpeed, double xSpeed, double zRotation, okapi::QAngle angle) {
        ySpeed = std::clamp(ySpeed, -1.0, 1.0);
        xSpeed = -std::clamp(xSpeed, -1.0, 1.0);
        zRotation = std::clamp(zRotation, -1.0, 1.0);
        double fwd = ySpeed * std::cos(angle.convert(okapi::radian)) - xSpeed * std::sin(angle.convert(okapi::radian));
        double right = ySpeed * std::sin(angle.convert(okapi::radian)) + xSpeed * std::cos(angle.convert(okapi::radian));

        double frontLeft = fwd - right + zRotation;
        double frontRight = fwd + right - zRotation;
        double backLeft = fwd + right + zRotation;
        double backRight = fwd - right - zRotation;

        double max = std::max(std::max(std::abs(frontLeft), std::abs(frontRight)), std::max(std::abs(backLeft), std::abs(backRight)));
        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        return HolonomicWheelSpeeds(frontLeft, frontRight, backLeft, backRight);
    }

}

}
