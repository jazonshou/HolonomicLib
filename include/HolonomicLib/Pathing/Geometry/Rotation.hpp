#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace HolonomicLib{

class Rotation{
    public:
    constexpr Rotation() = default;

    Rotation(okapi::QAngle itheta);

    Rotation(okapi::QLength ix, okapi::QLength iy);

    Rotation(double ix, double iy);

    ~Rotation() = default;

    okapi::QAngle Theta() const;

    double Sin() const;

    double Cos() const;

    double Tan() const;

    Rotation operator+(const Rotation &rhs) const;

    Rotation operator-(const Rotation &rhs) const;

    Rotation operator-() const;

    Rotation operator*(double scalar) const;

    Rotation operator/(double scalar) const;

    bool operator==(const Rotation &rhs) const;

    bool operator!=(const Rotation &rhs) const;

    void operator=(const Rotation &rhs);

    Rotation rotateBy(const Rotation &rhs) const;

    private:
    okapi::QAngle theta{0.0};
    double cosine{1.0};
    double sine{0.0};
};

}
