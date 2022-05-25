#pragma once
#include <initializer_list>
#include <vector>

namespace HolonomicLib {

/**
 * @brief Struct for trajectory states
 * 
 */
struct TrajectoryState
{   
    /**
     * @brief Construct a new Trajectory State object
     * 
     */
    TrajectoryState() = default;

    /**
     * @brief Construct a new Trajectory State object
     * 
     * @param ix desired x position
     * @param iy desired y position
     * @param itheta desired angle
     */
    TrajectoryState(double ix, double iy, double itheta);

    /**
     * @brief Construct a new Trajectory State object
     * 
     * @param ix desired x position
     * @param iy desired y position
     * @param itheta desired angle
     * @param ilinVel desired linear velocity
     * @param ilinAccel desired linear acceleration
     */
    TrajectoryState(double ix, double iy, double itheta, double ilinVel, 
                    double ilinAccel);

    double x, y, theta, linVel, linAccel;
};

/**
 * @brief Class for trajectory (vector of trajectory states)
 * 
 */
class Trajectory {
    private:
    std::vector<TrajectoryState> trajectory;

    public:
    /**
     * @brief Construct a new Trajectory object
     * 
     */
    Trajectory() = default;

    /**
     * @brief Construct a new Trajectory object
     * 
     * @param itrajectory list of trajectory states
     */
    Trajectory(const std::initializer_list<TrajectoryState> &itrajectory);

    /**
     * @brief Returns the trajectory state based on given index
     * 
     * @param index index of the trajectory
     * @return trajectory state at the index
     */
    TrajectoryState operator[](int index) const;

    /**
     * @brief Returns the size of the trajectory
     * 
     * @return size of the trajectory
     */
    int size() const;
};

}