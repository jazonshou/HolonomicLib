#include "ExpandedXDriveModel.hpp"

namespace HolonomicLib {

void ExpandedXDriveModel::cartesian(double xSpeed, 
                                    double ySpeed, 
                                    double zRotation, 
                                    okapi::QAngle angle, 
                                    double threshold) 
{
    ySpeed = -okapi::deadband(std::clamp(ySpeed, -1.0, 1.0), -std::abs(threshold), std::abs(threshold));
    xSpeed = okapi::deadband(std::clamp(xSpeed, -1.0, 1.0), -std::abs(threshold), std::abs(threshold));
    zRotation = okapi::deadband(std::clamp(zRotation, -1.0, 1.0), -std::abs(threshold), std::abs(threshold));
    double fwd = xSpeed * std::cos(angle.convert(okapi::radian)) - ySpeed * std::sin(angle.convert(okapi::radian));
    double right = xSpeed * std::sin(angle.convert(okapi::radian)) + ySpeed * std::cos(angle.convert(okapi::radian));

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
    topLeftMotor->moveVoltage(frontLeft * 12000);
    topRightMotor->moveVoltage(frontRight * 12000);
    bottomLeftMotor->moveVoltage(backLeft * 12000);
    bottomRightMotor->moveVoltage(backRight * 12000);
}

void ExpandedXDriveModel::polar(double magnitude, 
                                okapi::QAngle direction, 
                                double zRotation, 
                                okapi::QAngle angle, 
                                double threshold) 
{
    magnitude = okapi::deadband(std::clamp(magnitude, -1.0, 1.0), -std::abs(threshold), std::abs(threshold));
    zRotation = okapi::deadband(std::clamp(zRotation, -1.0, 1.0), -std::abs(threshold), std::abs(threshold));
    double fwd = magnitude * std::sin(direction.convert(okapi::radian));
    double right = magnitude * std::cos(direction.convert(okapi::radian));

    cartesian(fwd, right, zRotation, angle, threshold);
}

}