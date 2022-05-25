#pragma once
#include "okapi/api/units/QAcceleration.hpp"
#include "okapi/api/units/QSpeed.hpp"

namespace HolonomicLib {

struct FeedforwardGains {
    double kV, kA;
    FeedforwardGains() = default;
    FeedforwardGains(double ikV, double ikA);
};

class FeedforwardController {
    private:
    double kA, kV;

    public:
    FeedforwardController(double ikV);
    FeedforwardController(double ikV, double ikA);
    FeedforwardController(FeedforwardGains &gains);
    void setGain(double ikV, double ikA);
    double getkA() const;
    double getkV() const;
    double step(double vel, double accel);
};

}