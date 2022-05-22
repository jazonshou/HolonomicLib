#pragma once
#include "okapi/api/units/QAcceleration.hpp"
#include "okapi/api/units/QSpeed.hpp"

namespace HolonomicLib {

class FeedforwardController {
    private:
    double kA, kV;

    public:
    FeedforwardController(double ikV);
    FeedforwardController(double ikV, double ikA);
    void setGain(double ikV, double ikA);
    double getkA() const;
    double getkV() const;
    double step(double vel, double accel);
};

}