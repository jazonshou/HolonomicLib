#include "Pose2D.hpp"

namespace HolonomicLib {

Pose2D::Pose2D(okapi::QLength ix, okapi::QLength iy, okapi::QAngle itheta) {
    x = ix;
    y = iy;
    theta = itheta;
}

}
