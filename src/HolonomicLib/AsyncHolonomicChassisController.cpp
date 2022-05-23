#include "AsyncHolonomicChassisController.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"

namespace HolonomicLib {

AsyncHolonomicChassisController::AsyncHolonomicChassisController(std::shared_ptr<okapi::OdomChassisController> ichassis, 
                                    std::shared_ptr<okapi::IterativeController<double, double>> ixController,
                                    std::shared_ptr<okapi::IterativeController<double, double>> iyController,
                                    std::shared_ptr<okapi::IterativeController<double, double>> iturnController,
                                    const okapi::TimeUtil& itimeUtil)
{
    chassis = std::move(ichassis);
    leftFrontMotor = std::move(std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel())->getTopLeftMotor());
    leftBackMotor = std::move(std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel())->getBottomLeftMotor());
    rightFrontMotor = std::move(std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel())->getTopRightMotor());
    rightBackMotor = std::move(std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel())->getBottomRightMotor());

    rate = std::move(itimeUtil.getRate());
    timer = std::move(itimeUtil.getTimer());

    xController = std::move(ixController);
    yController = std::move(iyController);
    turnController = std::move(iturnController);

    auto holonomicController = std::make_unique<HolonomicController>(xController, yController, turnController);
}

void AsyncHolonomicChassisController::setTarget(Pose2D &targetPose, bool waitUntilSettled)
{
    lock.take(5);
    setState(ChassisState::TRANSLATING);
    resetControllers();
    controllerFlipDisabled(false);

    xController->setTarget(targetPose.x.convert(okapi::foot));
    yController->setTarget(targetPose.y.convert(okapi::foot));
    turnController->setTarget(targetPose.theta.convert(okapi::radian));
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

void AsyncHolonomicChassisController::loop() {
    while(true) {
        lock.take(5);
        currentOdomState = chassis->getState();
        currentPose = {currentOdomState.x, currentOdomState.y, currentOdomState.theta};
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
                double xOutput = xController->step(currentOdomState.x.convert(okapi::foot));
                double yOutput = yController->step(currentOdomState.y.convert(okapi::foot));
                double thetaOutput = turnController->step(currentAngle.convert(okapi::radian));
                
                HolonomicWheelSpeeds speeds = HolonomicMath::move(yOutput, xOutput, thetaOutput, currentAngle);
                move(speeds);
                if(holonomicController->isSettled()) {
                    setState(ChassisState::IDLE);
                }
                break;
            }

            case ChassisState::PATHING:
            {
                TrajectoryState desiredState = trajectory[(int)(currentTime.convert(okapi::millisecond) / 10)];
                Pose2D desiredPose = {desiredState.x * okapi::inch, desiredState.y * okapi::inch, desiredState.theta * okapi::radian};
                HolonomicWheelSpeeds speeds = 
                    holonomicController->step(currentPose, desiredPose, desiredState.linVel * okapi::ftps);
                move(speeds);

                if(currentTime > maxTime) {
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

void AsyncHolonomicChassisController::move(HolonomicWheelSpeeds &speeds) {
    leftFrontMotor->moveVoltage(speeds.frontLeft * 12000);
    leftBackMotor->moveVoltage(speeds.backLeft * 12000);
    rightFrontMotor->moveVoltage(speeds.frontRight * 12000);
    rightBackMotor->moveVoltage(speeds.backRight * 12000);
}

AsyncHolonomicChassisControllerBuilder&
AsyncHolonomicChassisControllerBuilder::withOutput(std::shared_ptr<okapi::OdomChassisController> ichassis) {
    chassis = std::move(ichassis);
    return *this;
}

}