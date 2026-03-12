#include <cstdio>
#include "miosix.h"
#include "arch/common/drivers/stm32f2_f4_i2c.h"
#include "BMP180.h"
#include "FlightEstimator.h"

using namespace miosix;

// --- HARDWARE PIN DEFINITIONS ---
typedef Gpio<GPIOB_BASE, 8> scl;
typedef Gpio<GPIOB_BASE, 9> sda;
typedef Gpio<GPIOD_BASE, 12> parachuteLed;

int main() {
    // 1. Hardware Initialization
    parachuteLed::mode(Mode::OUTPUT);
    parachuteLed::low(); // Ensure the LED is off at startup

    // Configure I2C1 pins (Alternate Function Open-Drain with internal Pull-Up)
    scl::mode(Mode::ALTERNATE_OD_PULL_UP);
    scl::alternateFunction(4);
    
    sda::mode(Mode::ALTERNATE_OD_PULL_UP);
    sda::alternateFunction(4);
    
    // Create the I2C bus hardware object with the specified pins
    I2C1Master i2c_bus(sda::getPin(), scl::getPin()); 

    printf("--- Sounding Rocket Initialization ---\n");

    // Instantiate the BMP180 sensor object (passing I2C bus and oversampling = 3)
    BMP180 bmpSensor(i2c_bus, 3);
    
    // --- SENSOR INITIALIZATION ---
    if (!bmpSensor.init()) {
        printf("CRITICAL ERROR: Sensor not found or calibration failed!\n");
        printf("Check SCL/SDA connections and 3V power supply.\n");
        
        // Busy wait loop to prevent WFI state and keep ST-Link active
        while(true) { 
            for(volatile int i = 0; i < 1000000; i++);
        } 
    }
    printf("BMP180 successfully initialized. EEPROM parameters read.\n");

    // Instantiate our flight computer
    FlightEstimator flightComputer;

    // 2. Calibration and Warm-up Phase
    printf("Warming up sensor and calibrating zero altitude...\n");
    
    // Discard the first 3 readings to stabilize the I2C bus
    for (int i = 0; i < 3; i++) {
        bmpSensor.sample(); 
    }

    // Read the current pressure to set the ground "Zero" reference
    bmpSensor.sample();
    Bmp180Sample groundData = bmpSensor.get_last_sample();
    flightComputer.calibrateZeroAltitude(groundData.pressure);
    
    printf("Calibration complete. Rocket on launchpad (State: IDLE).\n");

    printf("Time,Altitude,Velocity,State\n"); // CSV header for telemetry output

    const long long LOOP_PERIOD_NS = 50000000LL;

    // Initialize the timer for deltaTime calculation (getTime() returns nanoseconds)
    long long lastTime = getTime();
    long long absoluteNextWakeup = lastTime;
    float totalTime = 0.0f; 

    // 3. Operating System Main Loop
    while (true) {
        absoluteNextWakeup += LOOP_PERIOD_NS;
        // --- DATA ACQUISITION ---
        bmpSensor.sample();
        Bmp180Sample currentData = bmpSensor.get_last_sample();

        // --- ELAPSED TIME CALCULATION ---
        long long currentTime = getTime();
        float deltaTimeSec = (currentTime - lastTime) / 1000000000.0f; 
        lastTime = currentTime; 
        totalTime += deltaTimeSec; // Aggiorniamo il timestamp assoluto

        // --- DATA PROCESSING ---
        flightComputer.update(currentData.pressure, deltaTimeSec);

        // --- HARDWARE ACTUATION (Parachute) ---
        if (flightComputer.isApogeeReached()) {
            parachuteLed::high(); 
        } else {
            parachuteLed::low();  
        }

        // --- DEBUG AND TELEMETRY (Slowed down for human readability) ---
        printf("%.3f,%.3f,%.3f,%d\n",
               totalTime,
               flightComputer.getRelativeAltitude(),
               flightComputer.getVerticalVelocity(),
               (int)flightComputer.getState());
        
        // --- CPU YIELD ---
        Thread::nanoSleepUntil(absoluteNextWakeup);
    }
    
    return 0;
}