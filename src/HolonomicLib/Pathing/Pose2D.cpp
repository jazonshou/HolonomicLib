#include "Pose2D.hpp"

namespace HolonomicLib {

Pose2D::Pose2D(okapi::QLength ix, okapi::QLength iy, okapi::QAngle itheta) {
    x = ix;
    y = iy;
    theta = itheta;
}

Pose2D::Pose2D(const okapi::OdomState &iState){
    x = iState.x;
    y = -1 * iState.y;
    theta = Math::rescale180(iState.theta);
}

Pose2D::Pose2D(const TrajectoryState &iState){
    x = iState.x * okapi::foot;
    y = iState.y * okapi::foot;
    theta = iState.theta * okapi::degree;
}

Pose2D::Pose2D(const TimedTrajectoryState &iState){
    x = iState.x * okapi::foot;
    y = iState.y * okapi::foot;
    theta = iState.theta * okapi::degree;
}

okapi::QLength Pose2D::distanceTo(const Pose2D &other){
    return sqrt((x - other.x) * (x-other.x) + (y-other.y) * (y-other.y));
}

okapi::QAngle Pose2D::angleTo(const Pose2D &other){
    auto dx = other.x - x;
    auto dy = other.y - y;
    return atan2(dy, dx);
}

}
