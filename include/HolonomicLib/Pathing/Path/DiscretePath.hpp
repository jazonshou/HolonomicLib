#pragma once
#include "HolonomicLib/Pathing/Geometry/Translation.hpp"
#include "HolonomicLib/Utility/Units.hpp"
#include "HolonomicLib/Utility/Math.hpp"
#include<vector>
#include<iostream>
#include<cmath>

namespace HolonomicLib{

class DiscretePath{
    public:
    DiscretePath() = default;
    DiscretePath(const std::initializer_list<Point> &iWaypoint);
    DiscretePath(const std::vector<Point> &iWaypoint);
    ~DiscretePath() = default;

    DiscretePath operator+(const DiscretePath &rhs) const;
    DiscretePath operator+(const Point &rhs) const;
    DiscretePath& operator+=(const DiscretePath &rhs);
    DiscretePath& operator+=(const Point &rhs);

    Point getPoint(int index) const;
    Point operator[](int index) const;
    int size() const;

    private:
    std::vector<Point> path;
};
}