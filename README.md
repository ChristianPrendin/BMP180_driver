# STM32 Flight Computer & BMP180 Driver 🚀
Final joint project for the Advanced Operating Systems (AOS) and Embedded Systems (ES) courses.
This project implements a model rocket flight computer on an STM32F4Discovery board using the Miosix RTOS.

## Project Features
1. **AOS - BMP180 Device Driver:** A custom bare-metal C++ I2C driver to read raw pressure and temperature data from the BMP180 sensor, handling EEPROM calibration and math compensation.

2. **ES - Flight Computer:** A high-level logic module (FlightEstimator) that applies an Exponential Moving Average (EMA) filter to the raw data. It calculates relative altitude and vertical velocity to feed a State Machine.

3. **Apogee Detection:** The state machine detects the exact apogee of the flight (velocity crossing zero) and triggers a simulated parachute deployment (Green LED on PD12).


## Hardware Setup

* **Microcontroller:** STM32F4Discovery (ARM Cortex-M4)
* **Sensor:** BMP180 Barometric Sensor
* **I2C Pins:** PB8 (SCL) and PB9 (SDA)
* **Actuator:** Green LED on PD12
* **Telemetry:** Serial via PB10 (TX) and PB11 (RX) at 19200 baud.


## Authors
* **Christian Prendin**
* **Caterina Gerini**