#pragma once 
#include "okapi/api/control/iterative/iterativeController.hpp"
#include "okapi/api/units/QSpeed.hpp"

#include <memory>

#include "HolonomicWheelSpeeds.hpp"
#include "Pose2D.hpp"

namespace HolonomicLib {

class HolonomicController {
    private: 
    std::shared_ptr<okapi::IterativeController<double, double>> xController;
    std::shared_ptr<okapi::IterativeController<double, double>> yController;
    std::shared_ptr<okapi::IterativeController<double, double>> thetaController;

    public:
    HolonomicController(std::shared_ptr<okapi::IterativeController<double, double>> ixController,
                        std::shared_ptr<okapi::IterativeController<double, double>> iyController,
                        std::shared_ptr<okapi::IterativeController<double, double>> ithetaController);

    bool isSettled();

    HolonomicWheelSpeeds step(const Pose2D &currPose, const Pose2D &targetPose);

    HolonomicWheelSpeeds step(const Pose2D &currPose, const Pose2D &targetPose, okapi::QSpeed targetLinVel);
};
}