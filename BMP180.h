#ifndef BMP180_H
#define BMP180_H

#include "miosix.h"
#include "arch/common/drivers/stm32f2_f4_i2c.h" 

using namespace miosix;

struct Bmp180Sample {
    float temperature;
    float pressure;
};

class BMP180 {
public:
    BMP180(I2C1Master& i2c_bus, unsigned char oversampling = 0);
    
    bool init();
    void sample();
    Bmp180Sample get_last_sample() const;

private:
    I2C1Master& i2c;
    unsigned char oss;
    Bmp180Sample last_sample;

    short ac1, ac2, ac3;
    unsigned short ac4, ac5, ac6;
    short b1, b2, mb, mc, md;
    
    short read_short(unsigned char reg_address);
    unsigned short read_ushort(unsigned char reg_address);
};

#endif