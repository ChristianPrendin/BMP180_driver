# STM32 Flight Computer & BMP180 Driver 🚀

> **Final joint project for the Advanced Operating Systems (AOS) and Embedded Systems (ES) courses at Politecnico di Milano.**

This repository contains the complete software stack for a model sounding rocket. It implements a deterministic flight computer on an **STM32F4Discovery** evaluation board running the **Miosix RTOS**, alongside a custom bare-metal driver for the **BMP180** barometric sensor.

---

## 🛠️ Project Architecture

The system is cleanly separated into two main architectural layers:

### 1. Hardware Abstraction - `BMP180 Driver`
* **Custom I2C Integration:** A C++ driver built from scratch utilizing the Miosix `I2C1Master` HAL.
* **Non-Blocking Acquisition:** Uses `Thread::sleep()` to yield the CPU back to the OS scheduler during the sensor's internal Analog-to-Digital conversions (up to 25.5 ms), preventing CPU starvation.
* **Hardware Calibration:** Automatically reads and parses the 11 hardware-specific EEPROM calibration coefficients required for Bosch's complex data compensation algorithms.

### 2. Application Logic - `Flight Computer`
* **EMA Filtering:** Implements an Exponential Moving Average (EMA) low-pass filter to smooth the intrinsically noisy raw barometric altitude data.
* **Kinematics Engine:** Calculates the vertical velocity as the discrete-time derivative of the filtered altitude over a strictly constant loop period.
* **Deterministic RTOS Loop:** Eliminates scheduling jitter by using absolute timing (`Thread::nanoSleepUntil()`), enforcing a flawless 20Hz (50ms) control loop.
* **Kinematic State Machine:** A 4-state deterministic machine (`IDLE`, `ASCENDING`, `APOGEE`, `DESCENDING`). It detects the apogee precisely when the vertical velocity crosses zero.
* **Transient Rejection:** Utilizes a 3-tick confidence counter before officially declaring the apogee, preventing premature parachute deployments caused by mechanical shocks.

---

## 🔌 Hardware Setup

| Component | STM32F4Discovery Pin | Description |
| :--- | :--- | :--- |
| **BMP180 SCL** | `PB8` | I2C Clock (Configured as ALTERNATE-OD-PULLUP) |
| **BMP180 SDA** | `PB9` | I2C Data (Configured as ALTERNATE-OD-PULLUP) |
| **LED Actuator** | `PD12` | Green LED (Simulates Parachute Deployment) |
| **Serial TX** | `PB10` | USART3 TX (Rerouted to avoid audio DAC conflicts) |
| **Serial RX** | `PB11` | USART3 RX |

*Note: The serial interface operates at 19200 baud and requires an FTDI adapter connected to USART3 for debugging and telemetry.*

---

## 📂 Repository Structure

* `main.cpp` - System entry point, RTOS configuration, and the 20Hz deterministic control loop.
* `BMP180.h` / `BMP180.cpp` - Sensor device driver and I2C hardware interface.
* `FlightEstimator.h` / `FlightEstimator.cpp` - Kinematics, EMA filter, and the flight state machine.
* `plot_telemetry.py` - Python script for capturing and visualizing real-time Hardware-in-the-Loop (HIL) data.
* `plots/` - Directory containing the generated state-space and telemetry graphs.

---

## ⚙️ How to Build and Run

This repository contains the standalone source code for the flight computer and the sensor driver. To compile and flash it onto the STM32F4Discovery, you need to integrate it into the Miosix kernel environment.

1. **Setup the Miosix Environment:**
   Download the [Miosix kernel](https://miosix.org/wiki/index.php?title=Miosix_Toolchain) and ensure the toolchain is correctly installed (or use the provided Docker container for the AOS course).

2. **Integrate the Source Code:**
   Clone this repository and copy all the C++ source and header files (`main.cpp`, `BMP180.cpp`, `BMP180.h`, `FlightEstimator.cpp`, `FlightEstimator.h`) into the root directory of your Miosix project, replacing the default `main.cpp`.

3. **Update the Makefile:**
   Open the Miosix `Makefile` and ensure that all your custom `.cpp` files are included in the compilation sources. Look for the `SRC :=` variable and append your files:
   ```makefile
   SRC := main.cpp BMP180.cpp FlightEstimator.cpp
   ```

4. **Build and Flash:**
   Open your terminal in the Miosix directory and run the build command:
   ```bash
   make
   ```
   Once the `main.bin` file is generated, flash it to the STM32 board using the ST-Link utility:
   ```bash
   st-flash write main.bin 0x08000000
   ```
   *Note: If you encounter sleep-state issues with the ST-Link (due to the embedded RTOS sleep functions blocking the debug interface), hold the physical RESET button on the board, hit Enter to run the flash command, and release the button immediately as the writing process starts.*

---

## 📈 Telemetry & HIL Testing

To validate the embedded logic, the system streams real-time CSV data over the serial port. You can visualize the flight phases by running the provided Python tool:

1. Ensure the STM32 is connected via the FTDI adapter.
2. Install the required Python dependencies:
   ```bash
   pip install pyserial matplotlib
   ```
3. Run the telemetry script (adjust the COM port as necessary):
   ```bash
   python plot_telemetry.py --port COM3
   ```

---

## 👨‍💻 Authors

* **Christian Prendin**
* **Caterina Gerini**