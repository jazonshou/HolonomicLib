#include "Pose.hpp"
namespace HolonomicLib{

Pose::Pose(const Translation& itranslation, const Rotation& irotation){
    translation = itranslation;
    rotation = irotation;
}

Pose::Pose(okapi::QLength ix, okapi::QLength iy, const Rotation& irotation)
    : translation(ix, iy), rotation(irotation){}

Pose::Pose(const okapi::OdomState& istate){
    translation = Translation(istate.x, -1*istate.y);
    rotation = Rotation(istate.theta);
}

Pose::Pose(const TrajectoryState &istate){
    translation = Translation(istate.x * okapi::foot, istate.y * okapi::foot);
    rotation = Rotation(istate.theta * okapi::degree);
}

Pose::Pose(const TimedTrajectoryState &istate){
    translation = Translation(istate.x * okapi::foot, istate.y * okapi::foot);
    rotation = Rotation(istate.theta * okapi::degree);
}

const Translation& Pose::getTranslation() const{
    return translation;
}

const Rotation& Pose::getRotation() const{
    return rotation;
}

okapi::QLength Pose::X() const{
    return translation.X();
}

okapi::QLength Pose::Y() const{
    return translation.Y();
}

okapi::QAngle Pose::Theta() const{
    return rotation.Theta();
}

bool Pose::operator==(const Pose& rhs) const{
    return translation == rhs.translation && rotation == rhs.rotation;
}

bool Pose::operator!=(const Pose& rhs) const{
    return !operator==(rhs);
}

void Pose::operator=(const Pose& rhs){
    translation = getTranslation();
    rotation = getRotation();
}

okapi::QAngle Pose::angleTo(const Point& rhs) const{
    return rotation.Theta() - (rhs-translation).Theta();
}

}