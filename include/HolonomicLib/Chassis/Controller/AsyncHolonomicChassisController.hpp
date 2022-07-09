#pragma once

#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

#include "HolonomicLib/Pathing/Pose2D.hpp"
#include "HolonomicLib/Utility/Math.hpp"
#include "HolonomicLib/Utility/StateMachine.hpp"
#include "HolonomicLib/Utility/TaskWrapper.hpp"
#include "HolonomicLib/Utility/Units.hpp"


namespace HolonomicLib {

/**
 * @brief Enum for the different Chassis states
 *        FOLLOWING_PATH - The robot is following a path
 *        MOVING_TO_POINT - The robot is moving toward a point
 *        IDLE - No robot movement
 *
 */
enum class ChassisState { MOVING_TO_POINT, FOLLOWING_PATH, IDLE };

template class StateMachine<ChassisState>;

class AsyncHolonomicChassisController : public TaskWrapper, public StateMachine<ChassisState> {
    protected:
    /**
     * @brief Constructs a new Async Holonomic Chassis Controller object
     *
     * @param ichassis output chassis
     * @param idistController the distance PID controller
     * @param iturnController the turn PID Controller
     * @param itimeUtil okapi time utility
     */
    AsyncHolonomicChassisController(
      std::shared_ptr<okapi::OdomChassisController> ichassis,
      std::unique_ptr<okapi::IterativePosPIDController> idistController,
      std::unique_ptr<okapi::IterativePosPIDController> iturnController,
      const okapi::TimeUtil &itimeUtil);

    friend class AsyncHolonomicChassisControllerBuilder;

    public:
    /**
     * @brief Sets desired controller target (Pose)
     *
     * @param ipose desired Pose
     * @param waitUntilSettled if true, the controller will delay until the chassis has settled
     */
    void setTarget(const Pose2D &ipose, bool waitUntilSettled = false);

    /**
     * @brief Sets desired controller target (Trajectory - used with old Pathplanner)
     *
     * @param itrajectory trajectory to be followed
     * @param waitUntilSettled if true, the controller will delay until the chassis has settled
     */
    void setTarget(const Trajectory &itrajectory, bool waitUntilSettled = false);

    /**
     * @brief Sets desired controller target (TimedTrajectory - used with new Pathplanner)
     *
     * @param itrajectory trajectory to be followed
     * @param waitUntilSettled if true, the controller will delay until the chassis has settled
     */
    void setTarget(const TimedTrajectory &itrajectory, bool waitUntilSettled = false);

    /**
     * @brief Stops chassis
     *
     */
    void stop();

    /**
     * @brief Delays until the chassis has settled
     *
     */
    void waitUntilSettled();

    /**
     * @brief Sets current odom pose
     *
     * @param ipose current pose
     */
    void setPose(const Pose2D &ipose);

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
    /**
     * @brief TPeriodic task loop
     *
     */
    void loop() override;

    /**
     * @brief Resets the two PID controllers
     *
     */
    void resetControllers();

    /**
     * @brief Set the loop time of the two PID controllers
     *
     * @param itime the loop time
     */
    void setControllerSampleTime(okapi::QTime itime);

    std::shared_ptr<okapi::XDriveModel> model{nullptr};
    std::shared_ptr<okapi::OdomChassisController> chassis{nullptr};

    std::unique_ptr<okapi::IterativePosPIDController> distController{nullptr};
    std::unique_ptr<okapi::IterativePosPIDController> turnController{nullptr};

    std::unique_ptr<okapi::AbstractRate> rate;
    std::unique_ptr<okapi::AbstractTimer> timer;

    Trajectory trajectory;
    TimedTrajectory timedTrajectory;
    Pose2D endPose{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};

    Pose2D currentPose;
    bool isTimedTrajectory{false};
    int index{0};
    okapi::QTime delayTime;

    pros::Mutex lock;
};

class AsyncHolonomicChassisControllerBuilder {
    public:
    /**
     * @brief Constructs a new AsyncHolonomicChassisControllerBuilder object
     *
     * @param ichassis the chassis to output to
     */
    AsyncHolonomicChassisControllerBuilder(std::shared_ptr<okapi::OdomChassisController> ichassis);

    /**
     * @brief Destroys the AsyncHolonomicChassisControllerBuilder object
     *
     */
    ~AsyncHolonomicChassisControllerBuilder() = default;

    /**
     * @brief Sets the distance PID controller.
     *             (Note: settling parameters must be given)
     *
     * @param idistController the supplied PID controller
     * @return ongoing builder
     */
    AsyncHolonomicChassisControllerBuilder &
      withDistPID(std::unique_ptr<okapi::IterativePosPIDController> idistController);

    /**
     * @brief Sets the turn PID controller.
     *             (Note: settling parameters must be given)
     *
     * @param iturnController the supplied turn PID controller
     * @return ongoing builder
     */
    AsyncHolonomicChassisControllerBuilder &
      withTurnPID(std::unique_ptr<okapi::IterativePosPIDController> iturnController);

    /**
     * @brief Sets distance PID gains
     *
     * @param idistGains the supplied distance PID gains
     * @return ongoing builder
     */
    AsyncHolonomicChassisControllerBuilder &
      withDistGains(const okapi::IterativePosPIDController::Gains &idistGains);

    /**
     * @brief Sets turn PID gains
     *
     * @param iturnGains the supplied turn PID gains
     * @return ongoing builder
     */
    AsyncHolonomicChassisControllerBuilder &
      withTurnGains(const okapi::IterativePosPIDController::Gains &iturnGains);

    /**
     * @brief Sets settle parameters of the distance PID controller
     *
     * @param imaxError maximun error (tolerance)
     * @param imaxDerivative maximun derivative
     * @param iwaitTime the minimun time to be within imaxError to be considered settled
     * @return AsyncHolonomicChassisControllerBuilder& ongoing builder
     */
    AsyncHolonomicChassisControllerBuilder &
      withDistSettleParameters(okapi::QLength imaxError,
                               okapi::QSpeed imaxDerivative = 2 * okapi::inch / okapi::second,
                               okapi::QTime iwaitTime = 0.1 * okapi::second);

    /**
     * @brief Sets settle parameters of the turn PID controller
     *
     * @param imaxError maximun error (tolerance)
     * @param imaxDerivative maximun derivative
     * @param iwaitTime the minimun time to be within imaxError to be considered settled
     * @return ongoing builder
     */
    AsyncHolonomicChassisControllerBuilder &withTurnSettleParameters(
      okapi::QAngle imaxError,
      okapi::QAngularSpeed imaxDerivative = 10 * okapi::degree / okapi::second,
      okapi::QTime iwaitTime = 0.1 * okapi::second);

    /**
     * @brief Builds the AsyncHolonomicChassisController object. Note that in order to build, both
     *        the distance and the turn PID gains need to be supplied (or you can just pass a PID
     * controller)
     *
     * @return The built async controller with the given parameters
     */
    std::shared_ptr<AsyncHolonomicChassisController> build();

    private:
    std::shared_ptr<okapi::OdomChassisController> chassis;
    std::unique_ptr<okapi::IterativePosPIDController> distController;
    std::unique_ptr<okapi::IterativePosPIDController> turnController;

    bool distPIDInit{false};
    bool turnPIDInit{false};
};

} // namespace HolonomicLib