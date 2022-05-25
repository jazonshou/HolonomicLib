#pragma once
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace HolonomicLib {

class ExpandedXDriveModel : public okapi::XDriveModel {
    public: 
    /**
     * @brief Drives the chassis using cartesian inverse kinematics
     * 
     * @param xSpeed speed along x axis (positive = forward)
     * @param ySpeed speed along y axis (positive = right)
     * @param zRotation speed along z axis (positive = clockwise)
     * @param angle current robot angle
     * @param threshold deadband
     */
    void cartesian(double xSpeed, 
                   double ySpeed, 
                   double zRotation, 
                   okapi::QAngle angle, 
                   double threshold = 0);

    /**
     * @brief Drives the chassis using polar inverse kinematics
     * 
     * @param magnitude magnitude of the vector
     * @param direction direction of the vector
     * @param zRotation speed along the z axis (positive = clockwise)
     * @param angle current robot angle
     * @param threshold deadband
     */
    void polar(double magnitude, 
               okapi::QAngle direction, 
               double zRotation, 
               okapi::QAngle angle, 
               double threshold = 0);
};

}