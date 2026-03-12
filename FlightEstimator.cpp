#include "FlightEstimator.h"

FlightEstimator::FlightEstimator(float emaAlpha, float seaLevelPressure)
    : state(FlightState::IDLE),
      alpha(emaAlpha),
      p0(seaLevelPressure),
      groundAltitudeOffset(0.0f),
      currentFilteredAltitude(0.0f),
      previousFilteredAltitude(0.0f),
      verticalVelocity(0.0f),
      maxAltitudeReached(0.0f),
      apogeeConfidenceCounter(0),
      landingConfidenceCounter(0) {}

void FlightEstimator::calibrateZeroAltitude(float currentPressurePa) {
    // Calculate the absolute altitude based on current weather/location
    groundAltitudeOffset = calculateAbsoluteAltitude(currentPressurePa);
    resetFlight();
}

void FlightEstimator::resetFlight() {
    state = FlightState::IDLE;
    currentFilteredAltitude = 0.0f;
    previousFilteredAltitude = 0.0f;
    verticalVelocity = 0.0f;
    maxAltitudeReached = 0.0f;
    apogeeConfidenceCounter = 0;
    landingConfidenceCounter = 0;
}

float FlightEstimator::calculateAbsoluteAltitude(float pressurePa) const {
    // International Barometric Formula: h = 44330 * (1 - (p / p0)^(1 / 5.255))
    return 44330.0f * (1.0f - std::pow(pressurePa / p0, 0.190295f));
}

void FlightEstimator::update(float pressurePa, float deltaTimeSec) {
    if (deltaTimeSec <= 0.0f) return; // Safety check to prevent division by zero

    // 1. Calculate raw relative altitude
    float absoluteAlt = calculateAbsoluteAltitude(pressurePa);
    float rawRelativeAlt = absoluteAlt - groundAltitudeOffset;

    // 2. Apply Exponential Moving Average (EMA) Low-Pass Filter
    currentFilteredAltitude = (alpha * rawRelativeAlt) + ((1.0f - alpha) * previousFilteredAltitude);

    // 3. Calculate Vertical Velocity (Derivative of altitude over time)
    verticalVelocity = (currentFilteredAltitude - previousFilteredAltitude) / deltaTimeSec;

    // Keep track of the maximum altitude reached
    if (currentFilteredAltitude > maxAltitudeReached) {
        maxAltitudeReached = currentFilteredAltitude;
    }

    // 4. State Machine Logic
    switch (state) {
        case FlightState::IDLE:
            // Takeoff threshold: Must be > X meters high AND ascending faster than Y m/s
            // This prevents noise from triggering a false launch.
            if (currentFilteredAltitude > 0.5f && verticalVelocity > 0.3f) {
                state = FlightState::ASCENDING;
            }
            break;

        case FlightState::ASCENDING:
            // Apogee detection: Velocity drops below 0 (rocket starts falling)
            if (verticalVelocity < 0.0f) {
                apogeeConfidenceCounter++;
                // Require 3 consecutive negative velocity readings to trigger apogee
                if (apogeeConfidenceCounter >= 3) { 
                    state = FlightState::APOGEE;
                }
            } else {
                // If velocity goes positive again, reset the counter (it was just noise)
                apogeeConfidenceCounter = 0; 
            }
            break;

        case FlightState::APOGEE:
            // The APOGEE state is an event. It lasts for exactly one update cycle, 
            // then it automatically transitions to DESCENDING.
            state = FlightState::DESCENDING;
            break;

        case FlightState::DESCENDING:
            // Rocket is falling. Check if it has landed.
            if (currentFilteredAltitude < 10.0f && std::abs(verticalVelocity) < 0.5f) {
                landingConfidenceCounter++;
                if (landingConfidenceCounter >= 40) { // Require stable ground reading for a bit
                    resetFlight(); // Auto-reset for the next launch!
                }
            } else {
                landingConfidenceCounter = 0;
            }
            break;
    }

    // 5. Prepare variables for the next iteration
    previousFilteredAltitude = currentFilteredAltitude;
}

// --- Getter Methods ---

float FlightEstimator::getRelativeAltitude() const {
    return currentFilteredAltitude;
}

float FlightEstimator::getVerticalVelocity() const {
    return verticalVelocity;
}

FlightEstimator::FlightState FlightEstimator::getState() const {
    return state;
}

bool FlightEstimator::isApogeeReached() const {
    // The main program can use this flag to trigger the LED
    return (state == FlightState::APOGEE || state == FlightState::DESCENDING);
}