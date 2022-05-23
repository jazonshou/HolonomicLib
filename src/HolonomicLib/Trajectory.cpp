#include "Trajectory.hpp"
#include <iostream>

namespace HolonomicLib {

TrajectoryState::TrajectoryState(double ix, double iy, double itheta, double ilinVel) : 
x(ix), y(iy), theta(itheta), linVel(ilinVel) {}

Trajectory::Trajectory(const std::initializer_list<TrajectoryState> &itrajectory) {
    trajectory = itrajectory;
}

int Trajectory::size() const {
    return trajectory.size();
}

TrajectoryState Trajectory::operator[](int index) const {
    if(index < 0 || index >= trajectory.size()) {
        return {0, 0, 0, 0};
    } 
    return trajectory[index];
}

}