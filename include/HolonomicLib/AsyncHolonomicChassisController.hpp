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
#include "HolonomicController.hpp"
#include "Units.hpp"
#include "ExpandedXDriveModel.hpp"

namespace HolonomicLib {

enum class ChassisState {
    PATHING, TRANSLATING, IDLE
};

template class StateMachine<ChassisState>;

class AsyncHolonomicChassisController : public TaskWrapper, 
                                        public StateMachine<ChassisState> 
{
    protected: 
    AsyncHolonomicChassisController(std::shared_ptr<okapi::OdomChassisController> ichassis,
                                    std::shared_ptr<okapi::IterativePosPIDController> ixController,
                                    std::shared_ptr<okapi::IterativePosPIDController> iyController,
                                    std::shared_ptr<okapi::IterativePosPIDController> iturnController,
                                    const okapi::TimeUtil& itimeUtil);
    
    friend class AsyncHolonomicChassisControllerBuilder;

    public: 
    void setTarget(Pose2D targetPose, bool waitUntilSettled = false);
    void setTarget(Trajectory &itrajectory, bool waitUntilSettled = false);
    void stop();
    void waitUntilSettled();
    void setPose(Pose2D &ipose);
    Pose2D getPose();

    protected:
    std::shared_ptr<ExpandedXDriveModel> model;
    std::shared_ptr<okapi::OdomChassisController> chassis;

    std::shared_ptr<okapi::IterativePosPIDController> xController{nullptr};
    std::shared_ptr<okapi::IterativePosPIDController> yController{nullptr};
    std::shared_ptr<okapi::IterativePosPIDController> turnController{nullptr};

    // std::unique_ptr<HolonomicController> holonomicController;

    std::unique_ptr<okapi::AbstractRate> rate;
    std::unique_ptr<okapi::AbstractTimer> timer;
    okapi::QTime maxTime{0.0};

    Trajectory trajectory;
    Pose2D initialPose{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};
    okapi::OdomState currentOdomState{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};
    Pose2D currentPose{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};

    pros::Mutex lock;

    void loop() override;

    private: 
    void resetControllers();
    void controllerFlipDisabled(bool isDisabled);

    void move(HolonomicWheelSpeeds &speeds);
};

class AsyncHolonomicChassisControllerBuilder {
    public: 
    AsyncHolonomicChassisControllerBuilder();
    ~AsyncHolonomicChassisControllerBuilder() = default;
    AsyncHolonomicChassisControllerBuilder& withOutput(std::shared_ptr<okapi::OdomChassisController> ichassis);
    AsyncHolonomicChassisControllerBuilder& withControllers(
                    std::shared_ptr<okapi::IterativePosPIDController> ixController,
                    std::shared_ptr<okapi::IterativePosPIDController> iyController,
                    std::shared_ptr<okapi::IterativePosPIDController> iturnController);
    std::shared_ptr<AsyncHolonomicChassisController> build();
    
    private:
    std::shared_ptr<okapi::OdomChassisController> chassis;
    std::shared_ptr<okapi::IterativePosPIDController> xController{nullptr};
    std::shared_ptr<okapi::IterativePosPIDController> yController{nullptr};
    std::shared_ptr<okapi::IterativePosPIDController> turnController{nullptr};
};

}