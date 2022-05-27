#include "AsyncHolonomicChassisController.hpp"

namespace HolonomicLib {

AsyncHolonomicChassisController::AsyncHolonomicChassisController(
                                    std::shared_ptr<okapi::OdomChassisController> ichassis,
                                    const okapi::IterativePosPIDController::Gains &itranslateGains,
                                    const okapi::IterativePosPIDController::Gains &iturnGains,
                                    const okapi::TimeUtil& itimeUtil)
{
    chassis = std::move(std::static_pointer_cast<okapi::OdomChassisController>(ichassis));
    model = std::static_pointer_cast<ExpandedXDriveModel>(ichassis->getModel());

    rate = std::move(itimeUtil.getRate());
    timer = std::move(itimeUtil.getTimer());

    xController = std::make_unique<okapi::IterativePosPIDController>(itranslateGains, itimeUtil);
    yController = std::make_unique<okapi::IterativePosPIDController>(itranslateGains, itimeUtil);
    turnController = std::make_unique<okapi::IterativePosPIDController>(iturnGains, itimeUtil);
    
    chassis->startOdomThread();
    chassis->setDefaultStateMode(okapi::StateMode::FRAME_TRANSFORMATION);
}

AsyncHolonomicChassisController::AsyncHolonomicChassisController(
                                    std::shared_ptr<okapi::OdomChassisController> ichassis,
                                    const okapi::IterativePosPIDController::Gains &itranslateGains,
                                    const okapi::IterativePosPIDController::Gains &iturnGains,
                                    const Pose2D &isettleTolerance,
                                    const okapi::TimeUtil& itimeUtil) : 
AsyncHolonomicChassisController(ichassis, itranslateGains, iturnGains, itimeUtil)
{
    settleTolerance = isettleTolerance;
}

// imagine this doesn't exist
AsyncHolonomicChassisController::AsyncHolonomicChassisController(
                                    std::shared_ptr<okapi::OdomChassisController> ichassis,
                                    const okapi::IterativePosPIDController::Gains &itranslateGains,
                                    const okapi::IterativePosPIDController::Gains &iturnGains,
                                    const FeedforwardGains &itranslateFFGains,
                                    const okapi::TimeUtil& itimeUtil)
{
    chassis = std::move(std::static_pointer_cast<okapi::OdomChassisController>(ichassis));
    model = std::static_pointer_cast<ExpandedXDriveModel>(ichassis->getModel());

    rate = std::move(itimeUtil.getRate());
    timer = std::move(itimeUtil.getTimer());

    xController = std::make_unique<okapi::IterativePosPIDController>(itranslateGains.kP, 
                                                                     itranslateGains.kI, 
                                                                     itranslateGains.kD, 
                                                                     itranslateGains.kBias,
                                                                     itimeUtil);
    yController = std::make_unique<okapi::IterativePosPIDController>(itranslateGains.kP, 
                                                                     itranslateGains.kI, 
                                                                     itranslateGains.kD, 
                                                                     itranslateGains.kBias,
                                                                     itimeUtil);
    turnController = std::make_unique<okapi::IterativePosPIDController>(iturnGains.kP, 
                                                                        iturnGains.kI, 
                                                                        iturnGains.kD, 
                                                                        iturnGains.kBias,
                                                                        itimeUtil);

    // translateFFController = std::make_unique<FeedforwardController>(itranslateFFGains.kV, 
    //                                                                 itranslateFFGains.kA);
    // customFF = true;
    
    chassis->startOdomThread();
    chassis->setDefaultStateMode(okapi::StateMode::FRAME_TRANSFORMATION);
}

void AsyncHolonomicChassisController::setTarget(Pose2D targetPose, bool waitUntilSettled)
{
    lock.take(5);
    setState(ChassisState::TRANSLATING);
    resetControllers();
    controllerFlipDisabled(false);

    endPose = {targetPose.x, targetPose.y, targetPose.theta};
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
    endPose = {trajectory[trajectory.size() - 1].x * okapi::foot, 
               trajectory[trajectory.size() - 1].y * okapi::foot, 
               trajectory[trajectory.size() - 1].theta * okapi::degree};
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

bool AsyncHolonomicChassisController::isSettled() {
    return 
    std::abs(std::abs(currentPose.x.convert(okapi::inch)) - std::abs(endPose.x.convert(okapi::inch))) < settleTolerance.x.convert(okapi::inch) &&
    std::abs(std::abs(currentPose.y.convert(okapi::inch)) - std::abs(endPose.y.convert(okapi::inch))) < settleTolerance.y.convert(okapi::inch) &&
    std::abs(std::abs(currentPose.theta.convert(okapi::degree)) - std::abs(endPose.theta.convert(okapi::degree))) < settleTolerance.theta.convert(okapi::degree);
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
                xController->setTarget(endPose.x.convert(okapi::inch));
                yController->setTarget(endPose.y.convert(okapi::inch));
                turnController->setTarget(endPose.theta.convert(okapi::degree));

                double xOutput = xController->step(currentOdomState.x.convert(okapi::inch));
                double yOutput = yController->step(currentOdomState.y.convert(okapi::inch));
                double thetaOutput = turnController->step(currentAngle.convert(okapi::degree));

                model->cartesian(xOutput, yOutput, thetaOutput, currentAngle);

                if(isSettled()) {
                    setState(ChassisState::IDLE);
                }
                break;
            }

            case ChassisState::PATHING:
            {
                int index = (int)(currentTime.convert(okapi::millisecond) / 10) >= trajectory.size() ? 
                            trajectory.size() - 1 : (int)(currentTime.convert(okapi::millisecond) / 10);
                TrajectoryState desiredState = trajectory[index];
                
                Pose2D desiredPose = {desiredState.x * okapi::foot, desiredState.y * okapi::foot, desiredState.theta * okapi::degree};
                
                // a very unprofessional way of converting from ftps to inps
                double desiredVel = desiredState.linVel * 12; 
                double desiredAccel = desiredState.linAccel * 12; 
                
                xController->setTarget(desiredPose.x.convert(okapi::inch));
                double xFB = xController->step(currentPose.x.convert(okapi::inch));
                yController->setTarget(desiredPose.y.convert(okapi::inch));
                double yFB = yController->step(currentPose.y.convert(okapi::inch));
                turnController->setTarget(desiredPose.theta.convert(okapi::degree));
                double thetaFB = turnController->step(currentPose.theta.convert(okapi::degree));

                model->cartesian(xFB, yFB, thetaFB, currentAngle);
                
                // compares current position with last position
                if(isSettled()) {
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
    outputInit = true;
    chassis = std::move(ichassis);
    return *this;
}

AsyncHolonomicChassisControllerBuilder&
AsyncHolonomicChassisControllerBuilder::withPIDGains(const okapi::IterativePosPIDController::Gains &itranslateGains,
                                                     const okapi::IterativePosPIDController::Gains &iturnGains) 
{
    pidInit = true;
    pidTranslateGains = itranslateGains;
    pidTurnGains = iturnGains;
    return *this;
}

AsyncHolonomicChassisControllerBuilder&
AsyncHolonomicChassisControllerBuilder::withTolerance(const Pose2D &isettleTolerance) 
{
    settleTolerance = isettleTolerance;
    return *this;
}

std::shared_ptr<AsyncHolonomicChassisController>
AsyncHolonomicChassisControllerBuilder::build() 
{
    if(pidInit && outputInit) {
        std::shared_ptr<AsyncHolonomicChassisController> ret(new AsyncHolonomicChassisController(
        std::move(chassis),
        pidTranslateGains,
        pidTurnGains,
        settleTolerance,
        okapi::TimeUtilFactory::withSettledUtilParams(settleTolerance.x.convert(okapi::inch), 
                                                      2, 
                                                      100 * okapi::millisecond))
        );
        ret->startTask();
        return std::move(ret);
    } else {
        throw std::runtime_error("AsyncHolonomicChassisControllerBuilder: you must at least give pid gains and a controller");
    }
}

}