#pragma once
#include "DiscretePath.hpp"

namespace HolonomicLib{

class SimplePath{
    public:
    SimplePath() = default;
    SimplePath(const std::initializer_list<Point> &iWaypoint);
    ~SimplePath() = default;

    SimplePath& generate(int iStep, bool iEnd = true);
    SimplePath& generate(okapi::QLength iLength, bool iEnd = true);
    DiscretePath smoothen(double iSmoothWeight, okapi::QLength iTolerance);
    DiscretePath noSmoothen();

    private:
    std::vector<Point> waypoint;
};


}
