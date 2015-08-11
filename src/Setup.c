#include <bcm2835.h>
#include <stdio.h>

int init_all(void) {
    if (!bcm2835_init()) return -1;
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
    return 0;
}

int end_all(void) {
    bcm2835_i2c_end();
    return 0;
}
