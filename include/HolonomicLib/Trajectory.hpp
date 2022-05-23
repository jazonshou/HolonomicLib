#pragma once
#include <initializer_list>
#include <vector>

namespace HolonomicLib {

struct TrajectoryState
{
    TrajectoryState() = default;
    TrajectoryState(double ix, double iy, double itheta, double ilinVel);
    double x, y, theta, linVel;
};

class Trajectory {
    private:
    std::vector<TrajectoryState> trajectory;

    public:
    Trajectory() = default;
    Trajectory(const std::initializer_list<TrajectoryState> &itrajectory);
    TrajectoryState operator[](int index) const;
    int size() const;
};

}