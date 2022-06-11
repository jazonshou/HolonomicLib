#include "AsyncHolonomicChassisController.hpp"

namespace HolonomicLib {

AsyncHolonomicChassisController::AsyncHolonomicChassisController(
                                    std::shared_ptr<okapi::OdomChassisController> ichassis,
                                    std::unique_ptr<okapi::IterativePosPIDController> idistController,
                                    std::unique_ptr<okapi::IterativePosPIDController> iturnController,
                                    const okapi::TimeUtil& itimeUtil)
{
    chassis = std::move(ichassis);
    model = std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel());

    rate = std::move(itimeUtil.getRate());
    timer = std::move(itimeUtil.getTimer());

    distController = std::move(distController);
    turnController = std::move(turnController);
    
    chassis->setDefaultStateMode(okapi::StateMode::FRAME_TRANSFORMATION);
}

void AsyncHolonomicChassisController::setTarget(const Pose2D &ipose, bool waitUntilSettled)
{
    lock.take(5);
    setState(ChassisState::MOVING);
    resetControllers();
    endPose = ipose;
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncHolonomicChassisController::setTarget(const Trajectory &itrajectory, bool waitUntilSettled)
{
    lock.take(5);
    setState(ChassisState::PATHING);
    resetControllers();
    trajectory = itrajectory;
    isTimedTrajectory = false;
    index = 0;
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncHolonomicChassisController::setTarget(const TimedTrajectory &itrajectory, bool waitUntilSettled)
{
    lock.take(5);
    setState(ChassisState::PATHING);
    resetControllers();
    timedTrajectory = itrajectory;
    isTimedTrajectory = true;
    index = 0;
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncHolonomicChassisController::stop() {
    lock.take(5);
    model->stop();
    setState(ChassisState::IDLE);
    lock.give();
}

void AsyncHolonomicChassisController::waitUntilSettled() {
    while(getState() != ChassisState::IDLE) {
        pros::delay(10);
    }
}

void AsyncHolonomicChassisController::setPose(const Pose2D &ipose) {
    lock.take(5);
    chassis->setState({ipose.x, ipose.y, ipose.theta});
    lock.give();
}

Pose2D AsyncHolonomicChassisController::getPose() {
    lock.take(5);
    auto ret = currentPose;
    lock.give();
    return ret;
}

bool AsyncHolonomicChassisController::isSettled() {
    return distController->isSettled() && turnController->isSettled();
}

void AsyncHolonomicChassisController::loop() {
    while(true) {
        lock.take(5);
        if(getState() == ChassisState::IDLE){      
            lock.give();
            rate->delayUntil(10);
            continue;
        }

        if(isSettled()){
            lock.give();
            stop();
            rate->delayUntil(10);
            continue;
        }

        currentPose = chassis->getState();    
        Pose2D targetPose = currentPose; 
        delayTime = 10 * okapi::millisecond;

        if(getState() == ChassisState::MOVING){
            targetPose = endPose;
        }
        else if(getState() == ChassisState::PATHING){
            if(isTimedTrajectory){
                targetPose = timedTrajectory[std::min(index, timedTrajectory.size()-1)];
                if(index < timedTrajectory.size()-1){
                    delayTime = (timedTrajectory[index+1].time - timedTrajectory[index].time) * okapi::second;
                }
            }
            else{
                targetPose = trajectory[std::min(index, timedTrajectory.size()-1)];
            }

            index++;
        }

        auto distError = currentPose.distanceTo(endPose);
        auto angleError = Math::rescale180(endPose.theta - currentPose.theta); 
        double angleToTarget = currentPose.angleTo(targetPose).convert(okapi::radian);

        setControllerSampleTime(delayTime);
        double distOutput = distController->step(-distError.convert(okapi::inch));
        double turnOutput = turnController->step(-angleError.convert(okapi::degree));
        double xOutput = distOutput * cos(angleToTarget);
        double yOutput = turnOutput * sin(angleToTarget);

        model->fieldOrientedXArcade(xOutput, yOutput, turnOutput, currentPose.theta);

        lock.give();
        rate->delayUntil(delayTime);
    }
}

void AsyncHolonomicChassisController::resetControllers() {
    distController->reset();
    turnController->reset();
}

void AsyncHolonomicChassisController::setControllerSampleTime(okapi::QTime itime){
    distController->setSampleTime(delayTime);
    turnController->setSampleTime(delayTime);
}

AsyncHolonomicChassisControllerBuilder::AsyncHolonomicChassisControllerBuilder(std::shared_ptr<okapi::OdomChassisController> ichassis){
    chassis = std::move(ichassis);
    distController = std::make_unique<okapi::IterativePosPIDController>
                    (0.0, 0.0, 0.0, 0.0, okapi::TimeUtilFactory::withSettledUtilParams(0.5, 2.0, 100 * okapi::millisecond));
    turnController = std::make_unique<okapi::IterativePosPIDController>
                    (0.0, 0.0, 0.0, 0.0, okapi::TimeUtilFactory::withSettledUtilParams(1.0, 10.0, 100 * okapi::millisecond));
}

AsyncHolonomicChassisControllerBuilder& AsyncHolonomicChassisControllerBuilder::withDistPID
                        (std::unique_ptr<okapi::IterativePosPIDController> idistController){
    distPIDInit = true;
    distController = std::move(idistController);
    return *this;
}

AsyncHolonomicChassisControllerBuilder& AsyncHolonomicChassisControllerBuilder::withTurnPID
                        (std::unique_ptr<okapi::IterativePosPIDController> iturnController){
    turnPIDInit = true;
    turnController = std::move(iturnController);
    return *this;
}

AsyncHolonomicChassisControllerBuilder& AsyncHolonomicChassisControllerBuilder::withDistGains
                        (const okapi::IterativePosPIDController::Gains &idistGains){
    distPIDInit = true;
    turnController->setGains(idistGains);
    return *this;
}

AsyncHolonomicChassisControllerBuilder& AsyncHolonomicChassisControllerBuilder::withTurnGains
                        (const okapi::IterativePosPIDController::Gains &iturnGains){
    turnPIDInit = true;
    turnController->setGains(iturnGains);
    return *this;
}

AsyncHolonomicChassisControllerBuilder& AsyncHolonomicChassisControllerBuilder::withDistSettleParameters(okapi::QLength imaxError, 
                                                                                                         okapi::QSpeed imaxDerivative, 
                                                                                                         okapi::QTime iwaitTime){
    auto turnGain = turnController->getGains();
    turnController = std::make_unique<okapi::IterativePosPIDController>(turnGain, 
                                                                        okapi::TimeUtilFactory::withSettledUtilParams(
                                                                            imaxError.convert(okapi::inch),
                                                                            imaxDerivative.convert(okapi::inch / okapi::second),
                                                                            iwaitTime));
    return *this;
}

AsyncHolonomicChassisControllerBuilder& AsyncHolonomicChassisControllerBuilder::withTurnSettleParameters(okapi::QAngle imaxError, 
                                                                                                         okapi::QAngularSpeed imaxDerivative, 
                                                                                                         okapi::QTime iwaitTime){
    auto turnGain = turnController->getGains();
    turnController = std::make_unique<okapi::IterativePosPIDController>(turnGain, 
                                                                        okapi::TimeUtilFactory::withSettledUtilParams(
                                                                            imaxError.convert(okapi::degree),
                                                                            imaxDerivative.convert(okapi::degree / okapi::second),
                                                                            iwaitTime));
    return *this;
}

std::shared_ptr<AsyncHolonomicChassisController> AsyncHolonomicChassisControllerBuilder::build(){
    if(!distPIDInit) {
        throw std::runtime_error("AsyncHolonomicChassisControllerBuilder: Dist PID gains are not given");
    }

    if(!turnPIDInit) {
        throw std::runtime_error("AsyncHolonomicChassisControllerBuilder: Turn PID gains are not given");
    }

    if(std::dynamic_pointer_cast<okapi::XDriveModel>(chassis->getModel()) == nullptr){
        throw std::runtime_error("AsyncHolonomicChassisControllerBuilder: The supplied ChassisController is not built with an X Drive");
    }


    std::shared_ptr<AsyncHolonomicChassisController> ret(new AsyncHolonomicChassisController(
        std::move(chassis),
        std::move(distController),
        std::move(turnController),
        okapi::TimeUtilFactory::createDefault()));

    ret->startTask();
    return std::move(ret);
}

}