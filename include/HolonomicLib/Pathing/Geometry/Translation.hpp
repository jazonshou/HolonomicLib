/**
 * @file Point.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief  Point structs
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "Rotation.hpp"
#include "okapi/api/units/QArea.hpp"

namespace HolonomicLib{

class Translation{
    public:
    constexpr Translation() = default;

    Translation(okapi::QLength ix, okapi::QLength iy);

    Translation(okapi::QLength imag, const Rotation &iangle);

    Translation(const Translation &rhs);

    ~Translation() = default;

    okapi::QLength X() const;

    okapi::QLength Y() const;

    Translation operator+(const Translation &rhs) const;

    Translation operator-(const Translation &rhs) const;

    Translation operator-() const;

    Translation operator*(double scalar) const;

    okapi::QArea operator*(const Translation &rhs) const;

    Translation operator/(double scalar) const;

    bool operator==(const Translation &rhs) const;

    bool operator!=(const Translation &rhs) const;

    void operator=(const Translation &rhs);

    okapi::QAngle Theta() const;

    std::pair<double, double> norm() const;

    okapi::QLength distTo(const Translation &rhs) const;

    okapi::QAngle angleTo(const Translation &rhs) const;

    okapi::QLength mag() const;

    Translation rotateBy(const Rotation &rhs) const;

    void setX(okapi::QLength ix);

    void setY(okapi::QLength iy);
    
    private:
    okapi::QLength x = 0 * okapi::meter;
    okapi::QLength y = 0 * okapi::meter;
};

typedef Translation Point;
}

