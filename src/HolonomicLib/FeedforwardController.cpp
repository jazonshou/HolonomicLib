#include "FeedforwardController.hpp"

namespace HolonomicLib {

FeedforwardGains::FeedforwardGains(double ikV, double ikA) : kV(ikV), kA(ikA) 
{}

FeedforwardController::FeedforwardController(double ikV) {
    kA = 0.0;
    kV = ikV;
}

FeedforwardController::FeedforwardController(double ikV, double ikA) {
    kA = ikA;
    kV = ikV;
}

FeedforwardController::FeedforwardController(FeedforwardGains &gains) {
    kA = gains.kA;
    kV = gains.kV;
}

void FeedforwardController::setGain(double ikV, double ikA) {
    kA = ikA;
    kV = ikV;
}

double FeedforwardController::getkA() const {
    return kA;
}

double FeedforwardController::getkV() const {
    return kV;
}

double FeedforwardController::step(double vel, double accel) {
    return kV * vel + kA * accel;
}

}