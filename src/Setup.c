#include <bcm2835.h>
#include <stdio.h>

int end_all(void) {
    bcm2835_i2c_end();
    return 0;
}
