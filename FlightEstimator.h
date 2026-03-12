#ifndef FLIGHT_ESTIMATOR_H
#define FLIGHT_ESTIMATOR_H

#include <cmath>

class FlightEstimator {
public:
    /**
     * @brief State machine simulating the sounding rocket's flight phases.
     */
    enum class FlightState {
        IDLE,       // Rocket is stationary on the launch pad
        ASCENDING,  // Constant positive velocity detected (liftoff)
        APOGEE,     // Velocity transitions from positive to negative (parachute trigger)
        DESCENDING  // Rocket is falling/in recovery
    };

    /**
     * @brief Constructor for the flight estimator.
     * @param emaAlpha Low-Pass filter coefficient (between 0 and 1). Default is 0.15.
     * @param seaLevelPressure Reference pressure at sea level in Pascal.
     */
    FlightEstimator(float emaAlpha = 0.15f, float seaLevelPressure = 101325.0f);

    /**
     * @brief Reads the ground pressure before launch to set the relative "zero" altitude.
     * @param currentPressurePa Current pressure measured in Pascal.
     */
    void calibrateZeroAltitude(float currentPressurePa);

    /**
     * @brief Resets the state machine and kinematics for a new flight.
     * Must be called before a new launch if the board is not power-cycled.
     */
    void resetFlight();

    /**
     * @brief Updates the filter, altitude, velocity, and state machine.
     * This is the core function to be called at every loop iteration in main.
     * @param pressurePa Raw pressure just read from the sensor.
     * @param deltaTimeSec Time elapsed since the last update (in seconds).
     */
    void update(float pressurePa, float deltaTimeSec);

    // Getters for retrieving data from the main application
    float getRelativeAltitude() const;
    float getVerticalVelocity() const;
    FlightState getState() const;
    bool isApogeeReached() const;

private:
    /**
     * @brief Internal utility method to apply the International Barometric Formula.
     * @param pressurePa Pressure in Pascal.
     * @return Absolute altitude in meters.
     */
    float calculateAbsoluteAltitude(float pressurePa) const;

    // --- State Variables ---
    FlightState state;
    
    // --- Constants and Calibration ---
    float alpha;                // EMA filter coefficient
    float p0;                   // Sea level pressure (Pa)
    float groundAltitudeOffset; // Offset to zero the altitude at launch

    // --- Kinematic Data ---
    float currentFilteredAltitude;
    float previousFilteredAltitude;
    float verticalVelocity;
    
    // --- Apogee Detection Logic ---
    float maxAltitudeReached;
    int apogeeConfidenceCounter; // Prevents accidental triggers due to sensor noise
    int landingConfidenceCounter; // Used to detect when the rocket has safely landed
};

#endif // FLIGHT_ESTIMATOR_H