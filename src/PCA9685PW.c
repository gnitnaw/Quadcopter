void PCA9685PW_init(int i) {
    if (i != 0 && i != 1) {
        puts("MUST BE 0 or 1 !");
        return;
    }
    bcm2835_i2c_setSlaveAddress(PCA9685PW);

}
void PCA9685PWMFreq(int fd, float freq);
void pca9685PWMReset(int fd);
void pca9685PWMWrite(int fd, int pin, int on, int off);
void pca9685PWMRead(int fd, int pin, int *on, int *off);

void pca9685FullOn(int fd, int pin, int tf);
void pca9685FullOff(int fd, int pin, int tf);
