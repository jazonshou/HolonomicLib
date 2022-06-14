#include "Rotation.hpp"
namespace HolonomicLib{

Rotation::Rotation(okapi::QAngle itheta){
    theta = itheta;
    sine = (sin(itheta)).convert(okapi::number);
    cosine = (cos(itheta)).convert(okapi::number);
}

Rotation::Rotation(okapi::QLength ix, okapi::QLength iy){
    const auto magnitude = hypot(ix, iy);
    if (magnitude > 1e-6 * okapi::meter) {
        sine = (iy / magnitude).convert(okapi::number);
        cosine = (ix / magnitude).convert(okapi::number);
    } else {
        sine = 0.0;
        cosine = 1.0;
    }
    theta = std::atan2(sine, cosine) * okapi::radian;
}

Rotation::Rotation(double ix, double iy){
    Rotation(ix * okapi::meter, iy * okapi::meter);
}

okapi::QAngle Rotation::Theta() const{
    return theta;
}

double Rotation::Sin() const{
    return sine;
}

double Rotation::Cos() const{
    return cosine;
}

double Rotation::Tan() const{
    return sine / cosine;
}

Rotation Rotation::operator+(const Rotation &rhs) const{
     return rotateBy(rhs); 
}

Rotation Rotation::operator-(const Rotation &rhs) const{
    return *this + -rhs;
}

Rotation Rotation::operator-() const{
    return Rotation(theta * -1);
}   

Rotation Rotation::operator*(double scalar) const{
    return Rotation(theta * scalar);
}

Rotation Rotation::operator/(double scalar) const{
    return Rotation(theta / scalar);
}

bool Rotation::operator==(const Rotation &rhs) const{
    return std::hypot(cosine - rhs.cosine, sine - rhs.sine) < 1E-9;
}

bool Rotation::operator!=(const Rotation &rhs) const{
    return !(operator==(rhs));
}

void Rotation::operator=(const Rotation &rhs){
    theta = rhs.theta;
    sine = rhs.sine;
    cosine = rhs.cosine;

}

Rotation Rotation::rotateBy(const Rotation &rhs) const{
    return {(cosine * rhs.cosine - sine * rhs.sine) * okapi::meter, (cosine * rhs.sine + sine * rhs.cosine) * okapi::meter};
}
}