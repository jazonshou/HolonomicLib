#include "HolonomicController.hpp"
#include "Pose2D.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "Units.hpp"
#include "Math.hpp"

namespace HolonomicLib {

HolonomicController::HolonomicController(std::shared_ptr<okapi::IterativePositionController<double, double>> ixController,
                                         std::shared_ptr<okapi::IterativePositionController<double, double>> iyController,
                                         std::shared_ptr<okapi::IterativePositionController<double, double>> ithetaController) 
{
    xController = std::move(ixController);
    yController = std::move(iyController);
    thetaController = std::move(ithetaController);
}

bool HolonomicController::isSettled() {
    return xController->isSettled() && yController->isSettled() && thetaController->isSettled();
}

HolonomicWheelSpeeds HolonomicController::step(Pose2D &currPose, 
                                               Pose2D &targetPose, 
                                               okapi::QSpeed targetLinVel)
{
    double linVel = targetLinVel.convert(okapi::ftps);

    // Feedforward
    double xFF = linVel * std::cos(targetPose.theta.convert(okapi::radian));
    double yFF = linVel * std::sin(targetPose.theta.convert(okapi::radian));

    thetaController->setTarget(targetPose.theta.convert(okapi::radian));
    double thetaFF = thetaController->step(currPose.theta.convert(okapi::radian));

    // Feedback
    xController->setTarget(targetPose.x.convert(okapi::foot));
    double xFB = xController->step(currPose.x.convert(okapi::foot));
    yController->setTarget(targetPose.y.convert(okapi::foot));
    double yFB = yController->step(currPose.y.convert(okapi::foot));

    return HolonomicMath::move(xFF + xFB, yFF + yFB, thetaFF, currPose.theta);
}

}