#include "Math.hpp"
#include <algorithm>

namespace HolonomicLib {

namespace Math {
    std::pair<double, double> rotateVector(double x, double y, okapi::QAngle angle) {
        double cosA = std::cos(angle.convert(okapi::radian));
        double sinA = std::sin(angle.convert(okapi::radian));
        return std::make_pair<double, double> (x * cosA - y * sinA, x * sinA + y * cosA);
    }
}

namespace HolonomicMath {

    HolonomicWheelSpeeds move(double ySpeed, double xSpeed, double zRotation, okapi::QAngle angle) {
        ySpeed = std::clamp(ySpeed, -1.0, 1.0);
        xSpeed = std::clamp(xSpeed, -1.0, 1.0);
        zRotation = std::clamp(zRotation, -1.0, 1.0);
        std::pair<double, double> speeds = Math::rotateVector(xSpeed, ySpeed, -angle);

        double frontLeft = speeds.first + speeds.second + zRotation;
        double frontRight = speeds.first - speeds.second - zRotation;
        double backLeft = speeds.first - speeds.second + zRotation;
        double backRight = speeds.first + speeds.second - zRotation;
        return HolonomicWheelSpeeds(frontLeft, frontRight, backLeft, backRight);
    }

}

}
