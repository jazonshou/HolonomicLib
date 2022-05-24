#pragma once
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace HolonomicLib {

class ExpandedXDriveModel : public okapi::XDriveModel {
    public: 
    void cartesian(double xSpeed, 
                   double ySpeed, 
                   double zRotation, 
                   okapi::QAngle angle, 
                   double threshold = 0);

    void polar(double magnitude, 
               okapi::QAngle direction, 
               double zRotation, 
               okapi::QAngle angle, 
               double threshold = 0);
};

}