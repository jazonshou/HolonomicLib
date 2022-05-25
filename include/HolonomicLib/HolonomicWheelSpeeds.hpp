#pragma once

namespace HolonomicLib {

/**
 * @brief Struct for wheel speeds
 * 
 */
struct HolonomicWheelSpeeds
{
    double frontLeft, frontRight, backLeft, backRight;

    /**
     * @brief Construct a new Holonomic Wheel Speeds object
     * 
     */
    HolonomicWheelSpeeds() = default;

    /**
     * @brief Construct a new Holonomic Wheel Speeds object
     * 
     * @param frontleft front left motor speed
     * @param frontRight front right motor speed
     * @param backLeft back left motor speed
     * @param backRight back right motor speed
     */
    HolonomicWheelSpeeds(double frontleft, double frontRight, double backLeft, double backRight);
};


}