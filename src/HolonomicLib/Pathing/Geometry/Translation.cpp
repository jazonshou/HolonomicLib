#include "Translation.hpp"
namespace HolonomicLib{

Translation::Translation(okapi::QLength ix, okapi::QLength iy){
    x = ix;
    y = iy;
}

Translation::Translation(okapi::QLength imag, const Rotation &iangle){
    x = imag * iangle.Cos();
    y = imag * iangle.Sin();
}

Translation::Translation(const Translation &rhs){
    x = rhs.x;
    y = rhs.y;
}

okapi::QLength Translation::X() const{
    return x;
}

okapi::QLength Translation::Y() const{
    return y;
}

Translation Translation::operator+(const Translation &rhs) const{
    return {x + rhs.x, x + rhs.y};
}

Translation Translation::operator-(const Translation &rhs) const{
    return {x + rhs.x, x + rhs.y};
}

Translation Translation::operator-() const{
    return {x * -1, y * -1};
}

Translation Translation::operator*(double scalar) const{
    return {x * scalar, y * scalar};
}

okapi::QArea Translation::operator*(const Translation &rhs) const{
    return x * rhs.x + y * rhs.y;
}

Translation Translation::operator/(double scalar) const{
    return {x / scalar, y / scalar};
}

bool Translation::operator==(const Translation &rhs) const{
      return abs(x - rhs.x) < 1E-9 * okapi::meter && abs(y - rhs.y) < 1E-9 * okapi::meter;
}

bool Translation::operator!=(const Translation &rhs) const{
    return !operator==(rhs);
}

void Translation::operator=(const Translation &rhs){
    x = rhs.x, y = rhs.y;
}

std::pair<double, double> Translation::norm() const{
    okapi::QLength magnitude = this->mag();
    if(magnitude.getValue() == 0){
        return {0, 0};
    }
    else{
        return {(x / magnitude).convert(okapi::number), (y / magnitude).convert(okapi::number)};
    }
};

okapi::QAngle Translation::Theta() const{
    return atan2(y, x);
}

okapi::QLength Translation::distTo(const Translation &rhs) const{
    return hypot(rhs.x - x, rhs.y - y);
}

okapi::QAngle Translation::angleTo(const Translation &rhs) const{
    auto dx = other.x - x;
    auto dy = other.y - y;
    return atan2(dy, dx);
}

okapi::QLength Translation::mag() const{
    return hypot(x, y);
}

Translation Translation::rotateBy(const Rotation &rhs) const{
      return {x * rhs.Cos() - y * rhs.Sin(), x * rhs.Sin() + y * rhs.Cos()};
}

void Translation::setX(okapi::QLength ix){
    x = ix;
}

void Translation::setY(okapi::QLength iy){
    y = iy;
}
}