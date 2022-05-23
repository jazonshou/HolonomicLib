#include "AsyncHolonomicChassisController.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"

namespace HolonomicLib {

AsyncHolonomicChassisController::AsyncHolonomicChassisController(std::shared_ptr<okapi::OdomChassisController> ichassis, 
                                    std::shared_ptr<okapi::IterativePosPIDController> ixController,
                                    std::shared_ptr<okapi::IterativePosPIDController> iyController,
                                    std::shared_ptr<okapi::IterativePosPIDController> iturnController,
                                    const okapi::TimeUtil& itimeUtil)
{
    std::cout << "im in the controller constructor!" << std::endl;

    chassis = std::move(ichassis);
    std::cout << "moved chassis" << std::endl;
    leftFrontMotor = std::move((std::static_pointer_cast<okapi::XDriveModel>(chassis->getModel()))->getTopLeftMotor());
    leftBackMotor = std::move((std::static_pointer_cast<okapi::XDriveModel>(chassis->getModel()))->getBottomLeftMotor());
    rightFrontMotor = std::move((std::static_pointer_cast<okapi::XDriveModel>(chassis->getModel()))->getTopRightMotor());
    rightBackMotor = std::move((std::static_pointer_cast<okapi::XDriveModel>(chassis->getModel()))->getBottomRightMotor());
    std::cout << "moved motors" << std::endl;

    rate = std::move(itimeUtil.getRate());
    timer = std::move(itimeUtil.getTimer());

    std::cout << "did timers" << std::endl;

    xController = std::move(ixController);
    yController = std::move(iyController);
    turnController = std::move(iturnController);
    std::cout << "did controllers" << std::endl;
    
    //todo!!!!
    // chassis->startOdomThread();

    auto holonomicController = std::make_unique<HolonomicController>(xController, yController, turnController);
    std::cout << "made holonomic controller" << std::endl;
}

void AsyncHolonomicChassisController::setTarget(Pose2D targetPose, bool waitUntilSettled)
{
    std::cout << "i just set target" << std::endl;
    lock.take(5);
    std::cout << "i took the lock" << std::endl;
    setState(ChassisState::TRANSLATING);
    std::cout << "i set the state" << std::endl;
    resetControllers();
    std::cout << "i reset the controllers" << std::endl;
    controllerFlipDisabled(false);
    std::cout << "i flipped the controllers" << std::endl;

    xController->setTarget(targetPose.x.convert(okapi::foot));
    yController->setTarget(targetPose.y.convert(okapi::foot));
    turnController->setTarget(targetPose.theta.convert(okapi::radian));
    std::cout << "i set targets" << std::endl;
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
    std::cout << "in the loop" << std::endl;
    while(true) {
        lock.take(5);
        currentOdomState = chassis->getState();
        currentPose = {currentOdomState.x, currentOdomState.y, currentOdomState.theta};
        okapi::QAngle currentAngle = currentOdomState.theta;
        okapi::QTime currentTime = timer->getDtFromMark();

        switch(getState()) {
            case ChassisState::IDLE:
            {
                std::cout << "IDLE" << std::endl;
                controllerFlipDisabled(true);
                break;
            }
            
            case ChassisState::TRANSLATING:
            {
                std::cout << "TRANSLATING" << std::endl;
                double xOutput = xController->step(currentOdomState.x.convert(okapi::foot));
                double yOutput = yController->step(currentOdomState.y.convert(okapi::foot));
                double thetaOutput = turnController->step(currentAngle.convert(okapi::radian));
                std::cout << "stepped" << std::endl;
                
                HolonomicWheelSpeeds speeds = HolonomicMath::move(yOutput, xOutput, thetaOutput, currentAngle);
                std::cout << "calculated wheel speeds" << std::endl;

                move(speeds);
                std::cout << "moved" << std::endl;

                if(xController->isSettled() && yController->isSettled() && turnController->isSettled()) {
                    setState(ChassisState::IDLE);
                }
                std::cout << ":)" << std::endl;
                break;
            }

            case ChassisState::PATHING:
            {
                std::cout << "PATHING" << std::endl;
                TrajectoryState desiredState = trajectory[(int)(currentTime.convert(okapi::millisecond) / 10)];
                Pose2D desiredPose = {desiredState.x * okapi::inch, desiredState.y * okapi::inch, desiredState.theta * okapi::radian};
                HolonomicWheelSpeeds speeds = 
                    holonomicController->step(currentPose, desiredPose, desiredState.linVel * okapi::ftps);
                move(speeds);

                if(currentTime > maxTime && holonomicController->isSettled()) {
                    setState(ChassisState::IDLE);
                }
                break;
            }
        }
        std::cout << ":) almost there!" << std::endl;
        lock.give();
        rate->delayUntil(10);
        std::cout << "we made it!!! :)" << std::endl;
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
    
    std::cout << "just made ret" << std::endl;
    ret->startTask();
    std::cout << "built!" << std::endl;
    return std::move(ret);
}

}