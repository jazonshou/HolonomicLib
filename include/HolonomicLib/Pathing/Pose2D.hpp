#pragma once
#include "HolonomicLib/Pathing/Trajectory.hpp"
#include "HolonomicLib/Utility/Math.hpp"
#include "okapi/api/odometry/odomState.hpp"


namespace HolonomicLib {

/**
 * @brief Struct for position
 *
 */
struct Pose2D {
    okapi::QLength x, y;
    okapi::QAngle theta;

    /**
     * @brief Construct a new Pose 2D object
     *
     */
    Pose2D() = default;

    /**
     * @brief Construct a new Pose 2D object
     *
     * @param ix x
     * @param iy y
     * @param itheta angle
     */
    Pose2D(okapi::QLength ix, okapi::QLength iy, okapi::QAngle itheta);

    /**
     * @brief Converts an okapi OdomState into a Pose2D
     *        This is done by negating y coordinate and constraining theta to be between -180 and
     * 180
     *
     * @param iState the supplied OdomState
     */
    Pose2D(const okapi::OdomState &iState);

    /**
     * @brief Converts a TrajectoryState into a Pose2D
     *        This is done by adding units (feet / degree) to the trajectory state's x, y and theta
     *
     * @param iState the supplied TrajectoryStaet
     */
    Pose2D(const TrajectoryState &iState);

    /**
     * @brief Converts a TimedTrajectoryState into a Pose2D
     *        This is done by adding units (feet / degree) to the trajectory state's x, y and theta
     *
     * @param iState the suppliedTimeTrajectoryState
     */
    Pose2D(const TimedTrajectoryState &iState);

    /**
     * @brief Computes the euclidean distance between two poses
     *
     * @param other the pose to calculate distance between
     * @return distance between the two pose
     */
    okapi::QLength distanceTo(const Pose2D &other);

    /**
     * @brief Computes the euclidean angle between two poses
     *
     * @param other the pose to calculate angle between
     * @return angle to the other pose
     */
    okapi::QAngle angleTo(const Pose2D &other);
};

} // namespace HolonomicLib