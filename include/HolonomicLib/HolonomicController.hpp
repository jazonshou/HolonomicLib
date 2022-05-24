#pragma once 
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QLength.hpp"

#include <memory>

#include "HolonomicWheelSpeeds.hpp"
#include "Pose2D.hpp"
#include "ExpandedXDriveModel.hpp"
#include "Units.hpp"
#include "Math.hpp"

namespace HolonomicLib {

class HolonomicController {
    private: 
    std::shared_ptr<okapi::IterativePosPIDController> xController;
    std::shared_ptr<okapi::IterativePosPIDController> yController;
    std::shared_ptr<okapi::IterativePosPIDController> thetaController;

    public:
    HolonomicController(std::shared_ptr<okapi::IterativePosPIDController> &ixController,
                        std::shared_ptr<okapi::IterativePosPIDController> &iyController,
                        std::shared_ptr<okapi::IterativePosPIDController> &ithetaController);

    bool isSettled();

    HolonomicWheelSpeeds step(const Pose2D &currPose, const Pose2D &targetPose);

    void step(std::shared_ptr<ExpandedXDriveModel> model, 
              const Pose2D &currPose, 
              const Pose2D &targetPose, 
              okapi::QSpeed targetLinVel);
};
}