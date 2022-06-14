#pragma once
#include "Translation.hpp"
#include "HolonomicLib/Pathing/Path/Trajectory.hpp"
#include "okapi/api/odometry/odomState.hpp"

namespace HolonomicLib{

class Pose{
    public:
    /**
     * @brief Construct a new Pose 2D object
     * 
     */
    constexpr Pose() = default;

    
    Pose(const Translation &itranslation, const Rotation &irotation);

    /**
     * @brief Construct a new Pose 2D object
     * 
     * @param ix x 
     * @param iy y 
     * @param itheta angle
     */
    Pose(okapi::QLength ix, okapi::QLength iy, const Rotation &irotation);

    /**
     * @brief Converts an okapi OdomState into a Pose2D
     *        This is done by negating y coordinate and constraining theta to be between -180 and 180
     * 
     * @param istate the supplied OdomState
     */
    Pose(const okapi::OdomState &istate);

    /**
     * @brief Converts a TrajectoryState into a Pose2D
     *        This is done by adding units (feet / degree) to the trajectory state's x, y and theta
     * 
     * @param iState the supplied TrajectoryStaet
     */
    Pose(const TrajectoryState &istate);

    /**
     * @brief Converts a TimedTrajectoryState into a Pose2D
     *        This is done by adding units (feet / degree) to the trajectory state's x, y and theta
     * 
     * @param iState the suppliedTimeTrajectoryState
     */
    Pose(const TimedTrajectoryState &iState);

    ~Pose() = default;

    const Translation& getTranslation() const;

    const Rotation& getRotation() const;

    okapi::QLength X() const;

    okapi::QLength Y() const;

    okapi::QAngle Theta() const;
    
    bool operator==(const Pose &rhs) const;

    bool operator!=(const Pose &rhs) const;

    void operator=(const Pose &rhs);

    okapi::QAngle angleTo(const Translation &rhs) const;

    private:
    Translation translation;
    Rotation rotation;
};
}

