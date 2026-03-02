#include <cstdio>
#include "miosix.h"
#include "arch/common/drivers/stm32f2_f4_i2c.h"
#include "BMP180.h"

using namespace std;
using namespace miosix;

// Hardware pin configuration for I2C1 on STM32F4Discovery
// SCL (Clock) is on PB8, SDA (Data) is on PB9
typedef Gpio<GPIOB_BASE, 8> scl;
typedef Gpio<GPIOB_BASE, 9> sda;

int main() {
    // --- HARDWARE SETUP ---
    scl::mode(Mode::ALTERNATE_OD_PULL_UP);
    scl::alternateFunction(4); // AF4 is the internal STM32 code to enable I2C1
    
    sda::mode(Mode::ALTERNATE_OD_PULL_UP);
    sda::alternateFunction(4);
    
    // Create the I2C bus hardware object with the specified pins
    I2C1Master i2c_bus(sda::getPin(), scl::getPin()); 
    
    // Create the sensor object
    BMP180 sensor(i2c_bus, 3);
    
    printf("--- AOS: BMP180 Driver Start ---\n");
    
    // --- SENSOR INITIALIZATION ---
    if (!sensor.init()) {
        printf("CRITICAL ERROR: Sensor not found or calibration failed!\n");
        printf("Check SCL/SDA connections and 3V power supply.\n");
        while(true) { 
            for(volatile int i = 0; i < 1000000; i++);
        } 
    }
    
    printf("Sensor successfully initialized! Starting sampling...\n\n");
    
    // --- INFINITE SAMPLING LOOP ---
    while(true) {
        sensor.sample();
        Bmp180Sample data = sensor.get_last_sample();
        printf("Temperature: %.1f C  |  Pressure: %.0f Pa\n", data.temperature, data.pressure);
        Thread::sleep(1000);
    }
    
    return 0;
}