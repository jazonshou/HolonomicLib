#pragma once
#include "okapi/api/units/QAcceleration.hpp"
#include "okapi/api/units/QSpeed.hpp"

/** Currently not used */
namespace HolonomicLib {

/**
 * @brief Struct for FF gains
 * 
 */
struct FeedforwardGains {
    double kV, kA;
    FeedforwardGains() = default;
    FeedforwardGains(double ikV, double ikA);
};

/**
 * @brief FF controller class
 * 
 */
class FeedforwardController {
    private:
    double kA, kV;

    public:
    /**
     * @brief Construct a new Feedforward Controller object
     * 
     * @param ikV velocity gain
     */
    FeedforwardController(double ikV);

    /**
     * @brief Construct a new Feedforward Controller object
     * 
     * @param ikV velocity gain
     * @param ikA acceleration gain
     */
    FeedforwardController(double ikV, double ikA);

    /**
     * @brief Construct a new Feedforward Controller object
     * 
     * @param gains velocity and acceleration gains
     */
    FeedforwardController(FeedforwardGains &gains);

    /**
     * @brief Sets gains
     * 
     * @param ikV velocity gain
     * @param ikA acceleration gain
     */
    void setGain(double ikV, double ikA);

    /**
     * @brief Returns velocity gain
     * 
     * @return velocity gain
     */
    double getkA() const;

    /**
     * @brief Returns acceleration gain
     * 
     * @return acceleration gain
     */
    double getkV() const;

    /**
     * @brief Updates the controller
     * 
     * @param vel desired velocity
     * @param accel desired acceleration
     * @return output power based on controller
     */
    double step(double vel, double accel);
};

}