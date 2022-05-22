#pragma once
#include "StateMachine.hpp"
#include "TaskWrapper.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/control/iterative/iterativeController.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "Pose2D.hpp"
#include <memory>
#include "HolonomicWheelSpeeds.hpp"
#include "Math.hpp"
#include "okapi/api/units/QAngle.hpp"

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
                                    std::unique_ptr<okapi::IterativeController<double, double>> ixController,
                                    std::unique_ptr<okapi::IterativeController<double, double>> iyController,
                                    std::unique_ptr<okapi::IterativeController<double, double>> iturnController,
                                    const okapi::TimeUtil& itimeUtil);
    
    friend class AsyncHolonomicChassisControllerBuilder;

    public: 
    void setTarget(Pose2D &targetPose, bool waitUntilSettled = false);
    void stop();
    void waitUnitlSettled();

    protected:
    std::shared_ptr<okapi::OdomChassisController> chassis;
    std::shared_ptr<okapi::AbstractMotor> leftFrontMotor;
    std::shared_ptr<okapi::AbstractMotor> leftBackMotor;
    std::shared_ptr<okapi::AbstractMotor> rightFrontMotor;
    std::shared_ptr<okapi::AbstractMotor> rightBackMotor;

    std::unique_ptr<okapi::IterativeController<double, double>> xController{nullptr};
    std::unique_ptr<okapi::IterativeController<double, double>> yController{nullptr};
    std::unique_ptr<okapi::IterativeController<double, double>> turnController{nullptr};

    pros::Mutex lock;

    void loop() override;
};

class AsyncHolonomicChassisControllerBuilder {
    public: 
    AsyncHolonomicChassisControllerBuilder();
    AsyncHolonomicChassisControllerBuilder& withOutput(std::shared_ptr<OdomChassisController> ichassis);
    

};

}