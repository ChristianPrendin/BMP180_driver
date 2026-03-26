#pragma once

struct FlightConfig {
    // --- Sensor and Filter Configuration (ES) ---
    static constexpr float EMA_ALPHA             = 0.2f;       
    static constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;  

    // --- RTOS Configuration (AOS) ---
    static constexpr long long LOOP_PERIOD_NS    = 50000000;      

    // --- Barometric Formula Constants ---
    static constexpr float BAROMETRIC_COEFF      = 44330.0f;
    static constexpr float BAROMETRIC_EXP        = 0.190295f;

    // --- Takeoff Thresholds ---
    static constexpr float MIN_TAKEOFF_ALTITUDE_M  = 1.0f;
    static constexpr float MIN_TAKEOFF_VELOCITY_MS = 0.5f;

    // --- Apogee Thresholds ---
    static constexpr float APOGEE_VELOCITY_THRESH_MS = 0.0f;
    static constexpr int   APOGEE_CONFIDENCE_TICKS   = 3;

    // --- Landing Thresholds ---
    static constexpr float MAX_LANDING_ALTITUDE_M   = 10.0f;
    static constexpr float MAX_LANDING_VELOCITY_MS  = 1.0f;
    static constexpr int   LANDING_CONFIDENCE_TICKS = 40;
};