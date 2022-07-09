#include "Trajectory.hpp"
#include <iostream>

namespace HolonomicLib {

/** Trajectory State */
TrajectoryState::TrajectoryState(double ix, double iy, double itheta)
    : x(ix), y(iy), theta(itheta) {}

/** Timed Trajectory State */
TimedTrajectoryState::TimedTrajectoryState(double itime, double ix, double iy, double itheta)
    : time(itime), x(ix), y(iy), theta(itheta) {}

/** Trajectory */
Trajectory::Trajectory(const std::initializer_list<TrajectoryState> &itrajectory) {
    trajectory = itrajectory;
}

int Trajectory::size() const {
    return trajectory.size();
}

TrajectoryState Trajectory::operator[](int index) const {
    if (index < 0 || index >= trajectory.size()) {
        return {0, 0, 0};
    }
    return trajectory[index];
}

/** Timed Trajectory */
TimedTrajectory::TimedTrajectory(const std::initializer_list<TimedTrajectoryState> &itrajectory) {
    trajectory = itrajectory;
}

int TimedTrajectory::size() const {
    return trajectory.size();
}

TimedTrajectoryState TimedTrajectory::operator[](int index) const {
    if (index < 0 || index >= trajectory.size()) {
        return {0, 0, 0, 0};
    }
    return trajectory[index];
}

} // namespace HolonomicLib