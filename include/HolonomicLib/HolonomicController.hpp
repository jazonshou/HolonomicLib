#pragma once 
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "HolonomicWheelSpeeds.hpp"
#include "Pose2D.hpp"
#include "okapi/api/units/QSpeed.hpp"

namespace HolonomicLib {

class HolonomicController {
    private: 
    std::shared_ptr<okapi::IterativePositionController<double, double>> xController;
    std::shared_ptr<okapi::IterativePositionController<double, double>> yController;
    std::shared_ptr<okapi::IterativePositionController<double, double>> thetaController;

    public:
    HolonomicController(std::shared_ptr<okapi::IterativePositionController<double, double>> ixController,
                        std::shared_ptr<okapi::IterativePositionController<double, double>> iyController,
                        std::shared_ptr<okapi::IterativePositionController<double, double>> ithetaController);

    bool isSettled();

    HolonomicWheelSpeeds step(Pose2D &currPose, Pose2D &targetPose);

    HolonomicWheelSpeeds step(Pose2D &currPose, Pose2D &targetPose, okapi::QSpeed targetLinVel);
};
}