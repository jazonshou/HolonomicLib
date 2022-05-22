#include "AsyncHolonomicChassisController.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"

namespace HolonomicLib {

AsyncHolonomicChassisController::AsyncHolonomicChassisController(std::shared_ptr<okapi::OdomChassisController> ichassis, 
                                    std::unique_ptr<okapi::IterativeController<double, double>> ixController,
                                    std::unique_ptr<okapi::IterativeController<double, double>> iyController,
                                    std::unique_ptr<okapi::IterativeController<double, double>> iturnController,
                                    const okapi::TimeUtil& itimeUtil)
{
    chassis = std::move(ichassis);
    leftFrontMotor = std::move(std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel())->getTopLeftMotor());
    leftBackMotor = std::move(std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel())->getBottomLeftMotor());
    rightFrontMotor = std::move(std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel())->getTopRightMotor());
    rightBackMotor = std::move(std::static_pointer_cast<okapi::XDriveModel>(ichassis->getModel())->getBottomRightMotor());

    xController = std::move(ixController);
    yController = std::move(iyController);
    turnController = std::move(iturnController);
}

void AsyncHolonomicChassisController::setTarget(Pose2D &targetPose, bool waitUntilSettled)
{
    lock.take(5);
    setState(ChassisState::TRANSLATING);
    xController->flipDisable(false);
    yController->flipDisable(false);
    turnController->flipDisable(false);
    xController->setTarget(targetPose.x.convert(okapi::foot));
    yController->setTarget(targetPose.y.convert(okapi::foot));
    turnController->setTarget(targetPose.theta.convert(okapi::radian));
    lock.give();

    if(waitUntilSettled) {
        waitUnitlSettled();
    }
}

void AsyncHolonomicChassisController::stop() {
    lock.take(5);
    setState(ChassisState::IDLE);
    xController->flipDisable(true);
    yController->flipDisable(true);
    turnController->flipDisable(true);
    lock.give();
}

void AsyncHolonomicChassisController::loop() {
    while(true) {
        lock.take(5);
        okapi::OdomState currentPose = chassis->getState();
        okapi::QAngle currentAngle = currentPose.theta;

        switch(getState()) {
            case ChassisState::IDLE:
                break;
            
            case ChassisState::TRANSLATING:
                double xOutput = xController->step(currentPose.x.convert(okapi::foot));
                double yOutput = yController->step(currentPose.y.convert(okapi::foot));
                double thetaOutput = turnController->step(currentPose.theta.convert(okapi::radian));
                
                HolonomicWheelSpeeds speeds = HolonomicMath::move(yOutput, xOutput, thetaOutput, currentAngle);
                leftFrontMotor->moveVoltage(speeds.frontLeft * 12000);
                leftBackMotor->moveVoltage(speeds.backLeft * 12000);
                rightFrontMotor->moveVoltage(speeds.frontRight * 12000);
                rightBackMotor->moveVoltage(speeds.backRight * 12000);
                break;
        }

        lock.give();
    }
}

}