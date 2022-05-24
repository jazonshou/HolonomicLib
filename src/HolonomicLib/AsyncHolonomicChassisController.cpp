#include "AsyncHolonomicChassisController.hpp"

namespace HolonomicLib {

AsyncHolonomicChassisController::AsyncHolonomicChassisController(std::shared_ptr<okapi::OdomChassisController> ichassis, 
                                    std::shared_ptr<okapi::IterativePosPIDController> ixController,
                                    std::shared_ptr<okapi::IterativePosPIDController> iyController,
                                    std::shared_ptr<okapi::IterativePosPIDController> iturnController,
                                    const okapi::TimeUtil& itimeUtil)
{
    chassis = std::move(std::static_pointer_cast<okapi::OdomChassisController>(ichassis));
    model = std::static_pointer_cast<ExpandedXDriveModel>(ichassis->getModel());

    rate = std::move(itimeUtil.getRate());
    timer = std::move(itimeUtil.getTimer());

    std::cout << "did timers" << std::endl;

    xController = std::move(ixController);
    yController = std::move(iyController);
    turnController = std::move(iturnController);
    std::cout << "did controllers" << std::endl;
    
    //todo!!!!
    chassis->startOdomThread();
    chassis->setDefaultStateMode(okapi::StateMode::FRAME_TRANSFORMATION);

    // auto holonomicController = std::make_unique<HolonomicController>(xController, yController, turnController);
    std::cout << "made holonomic controller" << std::endl;
}

void AsyncHolonomicChassisController::setTarget(Pose2D targetPose, bool waitUntilSettled)
{
    lock.take(5);
    setState(ChassisState::TRANSLATING);
    resetControllers();
    controllerFlipDisabled(false);

    xController->setTarget(targetPose.x.convert(okapi::inch));
    yController->setTarget(targetPose.y.convert(okapi::inch));
    turnController->setTarget(targetPose.theta.convert(okapi::degree));
    lock.give();

    if(waitUntilSettled) this->waitUntilSettled();
}

void AsyncHolonomicChassisController::setTarget(Trajectory &itrajectory, bool waitUntilSettled)
{
    lock.take(5);
    setState(ChassisState::PATHING);
    resetControllers();
    controllerFlipDisabled(false);
    trajectory = itrajectory;
    initialPose = {currentOdomState.x, currentOdomState.y, currentOdomState.theta};
    maxTime = (trajectory.size() * 10 + 20) * okapi::millisecond;
    timer->placeMark();
    std::cout << "target set " << std::endl;
    lock.give();

    if(waitUntilSettled) this->waitUntilSettled();
}

void AsyncHolonomicChassisController::stop() {
    lock.take(5);
    setState(ChassisState::IDLE);
    lock.give();
}

void AsyncHolonomicChassisController::waitUntilSettled() {
    while(getState() != ChassisState::IDLE) {
        pros::delay(10);
    }
}

void AsyncHolonomicChassisController::setPose(Pose2D &ipose) {
    lock.take(5);
    chassis->setState({ipose.x, ipose.y, ipose.theta});
    lock.give();
}

Pose2D AsyncHolonomicChassisController::getPose() {
    return currentPose;
}

void AsyncHolonomicChassisController::loop() {
    std::cout << "in the loop" << std::endl;
    while(true) {
        lock.take(5);
        currentOdomState = chassis->getState();
        currentPose = {currentOdomState.x, currentOdomState.y, Math::rescale180(currentOdomState.theta)};
        okapi::QAngle currentAngle = currentOdomState.theta;
        okapi::QTime currentTime = timer->getDtFromMark();

        switch(getState()) {
            case ChassisState::IDLE:
            {
                controllerFlipDisabled(true);
                break;
            }
            
            case ChassisState::TRANSLATING:
            {
                double xOutput = xController->step(currentOdomState.x.convert(okapi::inch));
                double yOutput = yController->step(currentOdomState.y.convert(okapi::inch));
                double thetaOutput = turnController->step(currentAngle.convert(okapi::degree));
                pros::lcd::print(5, "X Output: %f", xOutput);
                pros::lcd::print(6, "Y Output: %f", yOutput);
                
                model->cartesian(xOutput, yOutput, thetaOutput, currentAngle);

                if(xController->isSettled() && yController->isSettled() && turnController->isSettled()) {
                    setState(ChassisState::IDLE);
                }
                break;
            }

            case ChassisState::PATHING:
            {
                TrajectoryState desiredState = trajectory[(int)(currentTime.convert(okapi::millisecond) / 10)];
                Pose2D desiredPose = {desiredState.x * okapi::foot, desiredState.y * okapi::foot, desiredState.theta * okapi::degree};
                double desiredVel = desiredState.linVel * 12; //todo!!!!!

                okapi::QAngle vectorAngle = okapi::atan2(desiredPose.y - currentPose.y, 
                                                         desiredPose.x - currentPose.x);
                double xFF = desiredVel * std::cos(vectorAngle.convert(okapi::radian));
                double yFF = desiredVel * std::sin(vectorAngle.convert(okapi::radian));

                turnController->setTarget(desiredPose.theta.convert(okapi::degree));
                double thetaFF = turnController->step(currentPose.theta.convert(okapi::degree));

                xController->setTarget(desiredPose.x.convert(okapi::inch));
                double xFB = xController->step(currentPose.x.convert(okapi::inch));
                yController->setTarget(desiredPose.y.convert(okapi::inch));
                double yFB = yController->step(currentPose.y.convert(okapi::inch));

                model->cartesian(xFB + xFF, yFB + yFF, thetaFF, currentAngle);

                if(currentTime > maxTime /*&& holonomicController->isSettled()*/) {
                    setState(ChassisState::IDLE);
                }
                break;
            }
        }
        lock.give();
        rate->delayUntil(10);
    }
}

void AsyncHolonomicChassisController::resetControllers() {
    xController->reset();
    yController->reset();
    turnController->reset();
}

void AsyncHolonomicChassisController::controllerFlipDisabled(bool isDisabled) {
    xController->flipDisable(isDisabled);
    yController->flipDisable(isDisabled);
    turnController->flipDisable(isDisabled);
}


AsyncHolonomicChassisControllerBuilder::AsyncHolonomicChassisControllerBuilder()
{}

AsyncHolonomicChassisControllerBuilder&
AsyncHolonomicChassisControllerBuilder::withOutput(std::shared_ptr<okapi::OdomChassisController> ichassis) {
    std::cout << "builder withOutput" << std::endl;
    chassis = std::move(ichassis);
    return *this;
}

AsyncHolonomicChassisControllerBuilder&
AsyncHolonomicChassisControllerBuilder::withControllers(
                    std::shared_ptr<okapi::IterativePosPIDController> ixController,
                    std::shared_ptr<okapi::IterativePosPIDController> iyController,
                    std::shared_ptr<okapi::IterativePosPIDController> iturnController) 
{
    std::cout << "builder withControllers" << std::endl;
    xController = std::move(ixController);
    yController = std::move(iyController);
    turnController = std::move(iturnController);
    return *this;
}

std::shared_ptr<AsyncHolonomicChassisController>
AsyncHolonomicChassisControllerBuilder::build() 
{
    std::cout << "builder build" << std::endl;
    std::shared_ptr<AsyncHolonomicChassisController> ret(new AsyncHolonomicChassisController(
        std::move(chassis),
        std::move(xController),
        std::move(yController),
        std::move(turnController), 
        okapi::TimeUtilFactory::createDefault())
    );
    ret->startTask();
    return std::move(ret);
}

}