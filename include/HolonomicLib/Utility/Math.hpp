#pragma once
#define _USE_MATH_DEFINES
#include "okapi/api/units/QAngle.hpp"
#include <utility>


namespace HolonomicLib {

/**
 * @brief Math functions
 *
 */
namespace Math {

/**
 * @brief Rescales an angle to [-180, 180] (degrees)
 *
 * @param angle angle to be rescaled
 * @return rescaled angle
 */
okapi::QAngle rescale180(okapi::QAngle angle);

/**
 * @brief Rescales an angle to [-180, 180] (degrees)
 *
 * @param angle angle to be rescaled
 * @return rescaled angle
 */
double rescale180(double angle);

} // namespace Math

} // namespace HolonomicLib