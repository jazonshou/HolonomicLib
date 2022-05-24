// #include "HolonomicController.hpp"

// namespace HolonomicLib {

// HolonomicController::HolonomicController(std::shared_ptr<okapi::IterativePosPIDController> &ixController,
//                                          std::shared_ptr<okapi::IterativePosPIDController> &iyController,
//                                          std::shared_ptr<okapi::IterativePosPIDController> &ithetaController) 
// {
//     std::cout << "constructing holonomic controller" << std::endl;
//     xController = std::make_shared<okapi::IterativePosPIDController>(ixController);
//     yController = std::make_shared<okapi::IterativePosPIDController>(iyController);
//     thetaController = std::make_shared<okapi::IterativePosPIDController>(ithetaController);
//     std::cout << "constructed" << std::endl;
// }

// bool HolonomicController::isSettled() {
//     return xController->isSettled() && yController->isSettled() && thetaController->isSettled();
// }

// void HolonomicController::step(std::shared_ptr<ExpandedXDriveModel> model,
//                                const Pose2D &currPose, 
//                                const Pose2D &targetPose, 
//                                okapi::QSpeed targetLinVel)
// {
//     std::cout << "holonomic controller step" << std::endl;
//     double linVel = targetLinVel.convert(okapi::inps);

//     // Feedforward
//     double xFF = linVel * std::cos(targetPose.theta.convert(okapi::radian));
//     double yFF = linVel * std::sin(targetPose.theta.convert(okapi::radian));
    
//     std::cout << "yee" << std::endl;

//     thetaController->setTarget(targetPose.theta.convert(okapi::degree));
//     std::cout << "boop" << std::endl;
//     double thetaFF = thetaController->step(currPose.theta.convert(okapi::degree));

//     std::cout << "feedback" << std::endl;

//     // Feedback
//     xController->setTarget(targetPose.x.convert(okapi::inch));
//     double xFB = xController->step(currPose.x.convert(okapi::inch));
//     yController->setTarget(targetPose.y.convert(okapi::inch));
//     double yFB = yController->step(currPose.y.convert(okapi::inch));
    
//     std::cout << "l;olololol" << std::endl;

//     model->cartesian(xFF + xFB, yFF + yFB, thetaFF, currPose.theta);
// }

// }