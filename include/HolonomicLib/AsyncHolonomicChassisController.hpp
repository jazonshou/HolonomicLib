#pragma once
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/units/QSpeed.hpp"

#include <iostream>

#include "StateMachine.hpp"
#include "TaskWrapper.hpp"
#include "Pose2D.hpp"
#include "HolonomicWheelSpeeds.hpp"
#include "Math.hpp"
#include "Trajectory.hpp"
#include "Units.hpp"

namespace HolonomicLib {

/**
 * @brief Enum for the different Chassis states 
 *        PATHING - The robot is following a path
 *        TRANSLATING - The robot is translating via PID
 *        IDLE - No robot movement
 * 
 */
enum class ChassisState {
    PATHING, TRANSLATING, IDLE
};

template class StateMachine<ChassisState>;

class AsyncHolonomicChassisController : public TaskWrapper, 
                                        public StateMachine<ChassisState> 
{
    protected: 
    /**
     * @brief Construct a new Async Holonomic Chassis Controller object
     * 
     * @param ichassis output chassis
     * @param itranslateGains movement PID gains
     * @param iturnGains turn PID gains
     * @param itimeUtil okapi time utility
     */
    AsyncHolonomicChassisController(std::shared_ptr<okapi::OdomChassisController> ichassis,
                                    const okapi::IterativePosPIDController::Gains &itranslateGains,
                                    const okapi::IterativePosPIDController::Gains &iturnGains,
                                    const okapi::TimeUtil& itimeUtil);

    /**
     * @brief Construct a new Async Holonomic Chassis Controller object
     * 
     * @param ichassis output chassis
     * @param itranslateGains movementPID gains
     * @param iturnGains turn PID gains
     * @param isettleTolerance settle tolerance
     * @param itimeUtil okapi time utility
     */
    AsyncHolonomicChassisController(std::shared_ptr<okapi::OdomChassisController> ichassis,
                                    const okapi::IterativePosPIDController::Gains &itranslateGains,
                                    const okapi::IterativePosPIDController::Gains &iturnGains,
                                    const Pose2D &isettleTolerance,
                                    const okapi::TimeUtil& itimeUtil);

   
    friend class AsyncHolonomicChassisControllerBuilder;

    public: 
    /**
     * @brief Sets desired controller target (Pose)
     * 
     * @param targetPose desired Pose
     * @param waitUntilSettled if true, the controller will delay until the chassis has settled
     */
    void setTarget(Pose2D targetPose, bool waitUntilSettled = false);

    /**
     * @brief Sets desired controller target (Trajectory - used with old Pathplanner)
     * 
     * @param itrajectory trajectory to be followed
     * @param waitUntilSettled if true, the controller will delay until the chassis has settled
     */
    void setTarget(Trajectory &itrajectory, bool waitUntilSettled = false);

    /**
     * @brief Sets desired controller target (TimedTrajectory - used with new Pathplanner)
     * 
     * @param itrajectory trajectory to be followed
     * @param waitUntilSettled if true, the controller will delay until the chassis has settled
     */
    void setTarget(TimedTrajectory &itrajectory, bool waitUntilSettled = false);

    /**
     * @brief Stops chassis
     * 
     */
    void stop();

    /**
     * @brief delays until the chassis has settled
     * 
     */
    void waitUntilSettled();

    /**
     * @brief Sets current odom pose
     * 
     * @param ipose current pose
     */
    void setPose(Pose2D &ipose);

    /**
     * @brief Gets current odom pose
     * 
     * @return current pose
     */
    Pose2D getPose();

    /**
     * @brief Checks if the chassis is settled
     * 
     * @return if chassis is settled or not
     */
    bool isSettled();

    protected:
    std::shared_ptr<okapi::XDriveModel> model;
    std::shared_ptr<okapi::OdomChassisController> chassis;

    std::unique_ptr<okapi::IterativePosPIDController> xController{nullptr};
    std::unique_ptr<okapi::IterativePosPIDController> yController{nullptr};
    std::unique_ptr<okapi::IterativePosPIDController> turnController{nullptr};

    std::unique_ptr<okapi::AbstractRate> rate;
    std::unique_ptr<okapi::AbstractTimer> timer;
    okapi::QTime delayTime{0.0};

    Trajectory trajectory; 
    Pose2D initialPose{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};
    okapi::OdomState currentOdomState{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};
    Pose2D currentPose{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};
    
    Pose2D endPose{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};
    Pose2D settleTolerance{1 * okapi::inch, 1 * okapi::inch, 1 * okapi::degree};

    TimedTrajectory timedTrajectory;
    bool timedTrajectoryEnabled{false};

    int index{0};

    bool initialRun{true};

    pros::Mutex lock;

    void loop() override;

    private: 
    void resetControllers();
    void controllerFlipDisabled(bool isDisabled);
};

class AsyncHolonomicChassisControllerBuilder {
    public: 
    AsyncHolonomicChassisControllerBuilder();
    ~AsyncHolonomicChassisControllerBuilder() = default;

    /**
     * @brief Adds output controller
     * 
     * @param ichassis output controller (MUST BE okapi::OdomChassisController)
     * @return 
     */
    AsyncHolonomicChassisControllerBuilder& withOutput(std::shared_ptr<okapi::OdomChassisController> ichassis);

    /**
     * @brief Adds PID gains
     * 
     * @param itranslateGains movement gains
     * @param iturnGains turn gains
     * @return 
     */
    AsyncHolonomicChassisControllerBuilder& withPIDGains(const okapi::IterativePosPIDController::Gains &itranslateGains, 
                                                         const okapi::IterativePosPIDController::Gains &iturnGains);

    /**
     * @brief Adds tolerance 
     * 
     * @param isettleTolerance pose tolerance
     * @return 
     */
    AsyncHolonomicChassisControllerBuilder& withTolerance(const Pose2D &isettleTolerance);
    // AsyncHolonomicChassisControllerBuilder& withFFGains(const FeedforwardGains &itranslateGains);
    std::shared_ptr<AsyncHolonomicChassisController> build();
    
    private:
    std::shared_ptr<okapi::OdomChassisController> chassis;
    okapi::IterativePosPIDController::Gains pidTranslateGains; 
    okapi::IterativePosPIDController::Gains pidTurnGains;
    Pose2D settleTolerance{1 * okapi::inch, 1 * okapi::inch, 1 * okapi::degree};

    bool pidInit{false};
    bool outputInit{false};
};

}