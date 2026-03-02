#include "BMP180.h"

const unsigned char BMP180_ADDR = 0xEE;

BMP180::BMP180(I2C1Master& i2c_bus, unsigned char oversampling) 
    : i2c(i2c_bus), oss(oversampling) {
    last_sample.temperature = 0.0f;
    last_sample.pressure = 0.0f;
}

// Private method to read a signed 16-bit integer (short) from the sensor
short BMP180::read_short(unsigned char reg_address) {
    unsigned char msb = 0;
    unsigned char lsb = 0;
    
    i2c.send(BMP180_ADDR, &reg_address, 1);
    
    unsigned char buffer[2] = {0, 0};
    i2c.recv(BMP180_ADDR, buffer, 2); 
    
    msb = buffer[0];
    lsb = buffer[1];
    
    return (short)((msb << 8) | lsb);
}

// Private method to read an unsigned 16-bit integer (unsigned short) from the sensor
unsigned short BMP180::read_ushort(unsigned char reg_address) {
    unsigned char msb = 0;
    unsigned char lsb = 0;
    
    i2c.send(BMP180_ADDR, &reg_address, 1);
    
    unsigned char buffer[2] = {0, 0};
    i2c.recv(BMP180_ADDR, buffer, 2);
    
    msb = buffer[0];
    lsb = buffer[1];
    
    return (unsigned short)((msb << 8) | lsb);
}

bool BMP180::init() {
    // Read the calibration coefficients from the sensor's EEPROM
    ac1 = read_short(0xAA);
    ac2 = read_short(0xAC);
    ac3 = read_short(0xAE);
    ac4 = read_ushort(0xB0);
    ac5 = read_ushort(0xB2);
    ac6 = read_ushort(0xB4);
    b1  = read_short(0xB6);
    b2  = read_short(0xB8);
    mb  = read_short(0xBA);
    mc  = read_short(0xBC);
    md  = read_short(0xBE);

    // None of the calibration coefficients should be 0 or 0xFFFF
    if (ac1 == 0 || ac1 == -1 ||
        ac2 == 0 || ac2 == -1 ||
        ac3 == 0 || ac3 == -1 ||
        ac4 == 0 || ac4 == 0xFFFF ||
        ac5 == 0 || ac5 == 0xFFFF ||
        ac6 == 0 || ac6 == 0xFFFF ||
        b1  == 0 || b1  == -1 ||
        b2  == 0 || b2  == -1 ||
        mb  == 0 || mb  == -1 ||
        mc  == 0 || mc  == -1 ||
        md  == 0 || md  == -1) {
        
        return false;
    }

    return true;
}

// Simple getter to return the last computed sample
Bmp180Sample BMP180::get_last_sample() const {
    return last_sample;
}

// Main function to sample and compute temperature and pressure
void BMP180::sample() {
    // --- PHASE 1: Read Uncompensated Temperature (UT) ---
    unsigned char reg_ctrl = 0xF4; // Control register address
    unsigned char cmd_temp = 0x2E; // Command to measure temperature
    
    // Write 0x2E into register 0xF4
    unsigned char buf_temp_cmd[2] = {reg_ctrl, cmd_temp};
    i2c.send(BMP180_ADDR, buf_temp_cmd, 2);
    
    // The datasheet requires a 4.5 ms wait.
    Thread::sleep(5); 
    
    // Read the result from registers 0xF6 (MSB) and 0xF7 (LSB)
    unsigned char reg_data = 0xF6;
    i2c.send(BMP180_ADDR, &reg_data, 1);
    unsigned char buf_temp[2] = {0, 0};
    i2c.recv(BMP180_ADDR, buf_temp, 2);
    
    // Reconstruct the 16-bit raw data
    long ut = (buf_temp[0] << 8) + buf_temp[1];


    // --- PHASE 2: Read Uncompensated Pressure (UP) ---
    // The command depends on the chosen oversampling (oss)
    unsigned char cmd_press = 0x34 + (oss << 6); 
    unsigned char buf_press_cmd[2] = {reg_ctrl, cmd_press};
    i2c.send(BMP180_ADDR, buf_press_cmd, 2);
    
    // The wait time changes based on the requested oversampling
    int delay_ms = 5;                 // Standard for oss=0 (max 4.5ms)
    if (oss == 1) delay_ms = 8;       // max 7.5ms
    else if (oss == 2) delay_ms = 14; // max 13.5ms
    else if (oss == 3) delay_ms = 26; // max 25.5ms
    Thread::sleep(delay_ms);
    
    // Read the result across 3 bytes: 0xF6 (MSB), 0xF7 (LSB), 0xF8 (XLSB)
    i2c.send(BMP180_ADDR, &reg_data, 1);
    unsigned char buf_press[3] = {0, 0, 0};
    i2c.recv(BMP180_ADDR, buf_press, 3);
    
    // Reconstruct the data using the datasheet formula
    long up = ((buf_press[0] << 16) + (buf_press[1] << 8) + buf_press[2]) >> (8 - oss);


    // --- PHASE 3: Temperature Compensation ---
    // Official Bosch Sensortec algorithm
    long x1 = ((ut - ac6) * ac5) >> 15;
    long x2 = (mc << 11) / (x1 + md);
    long b5 = x1 + x2;                  
    long t = (b5 + 8) >> 4;             
    
    // Save the real temperature in Celsius
    last_sample.temperature = (float)t / 10.0f; 


    // --- PHASE 4: Pressure Compensation ---
    // Official Bosch Sensortec algorithm
    long b6 = b5 - 4000;
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    long x3 = x1 + x2;
    long b3 = ((((long)ac1 * 4 + x3) << oss) + 2) >> 2;
    
    x1 = (ac3 * b6) >> 13; 
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    unsigned long b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;
    unsigned long b7 = ((unsigned long)up - b3) * (50000 >> oss);
    
    long p;
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    
    // Save the real measured pressure in Pascal (Pa)
    last_sample.pressure = (float)p;
} 