#include "Math.hpp"
#include <algorithm>

namespace HolonomicLib {

namespace Math {
    std::pair<double, double> rotateVector(double x, double y, double angle) {
        double cosA = std::cos(angle * (M_PI / 180.0));
        double sinA = std::sin(angle * (M_PI / 180.0));
        return std::make_pair<double, double> (x * cosA - y * sinA, x * sinA + y * cosA);
    }
}

namespace HolonomicMath {

    HolonomicWheelSpeeds move(double ySpeed, double xSpeed, double zRotation, double angle = 0) {
        ySpeed = std::clamp(ySpeed, -1.0, 1.0);
        xSpeed = std::clamp(xSpeed, -1.0, 1.0);
        std::pair<double, double> speeds = Math::rotateVector(xSpeed, ySpeed, -angle);
    
        double frontLeft = speeds.first + speeds.second + zRotation;
        double frontRight = speeds.first - speeds.second - zRotation;
        double backLeft = speeds.first - speeds.second + zRotation;
        double backRight = speeds.first + speeds.second - zRotation;
        return HolonomicWheelSpeeds(frontLeft, frontRight, backLeft, backRight);
    }

}

}
