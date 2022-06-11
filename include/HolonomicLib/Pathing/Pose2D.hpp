#pragma once
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"

namespace HolonomicLib {

/**
 * @brief Struct for position
 * 
 */
struct Pose2D
{
    okapi::QLength x, y;
    okapi::QAngle theta;

    /**
     * @brief Construct a new Pose 2D object
     * 
     */
    Pose2D() = default;

    /**
     * @brief Construct a new Pose 2D object
     * 
     * @param ix x 
     * @param iy y 
     * @param itheta angle
     */
    Pose2D(okapi::QLength ix, okapi::QLength iy, okapi::QAngle itheta);
};

}