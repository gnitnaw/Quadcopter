#include "I2CControl.h"

// Address for ADXL345
#define ADXL345_DEVID           0x00
#define ADXL345_THRESH_TAP      0x1D
#define ADXL345_OFSX            0x1E
#define ADXL345_OFSY            0x1F
#define ADXL345_OFSZ            0x20
#define ADXL345_DUR             0x21
#define ADXL345_LATENT          0x22
#define ADXL345_WINDOW          0x23
#define ADXL345_THRESH_ACT      0x24
#define ADXL345_THRESH_INACT    0x25
#define ADXL345_TIME_INACT      0x26
#define ADXL345_ACT_INACT_CTL   0x27
#define ADXL345_THRESH_FF       0x28
#define ADXL345_TIME_FF         0x29
#define ADXL345_TAP_AXES        0x2A
#define ADXL345_ACT_TAP_STATUS  0x2B
#define ADXL345_BW_RATE         0x2C
#define ADXL345_POWER_CTL       0x2D
#define ADXL345_INT_ENABLE      0x2E
#define ADXL345_INT_MAP         0x2F
#define ADXL345_INT_SOURCE      0x30
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_DATAX0          0x32
#define ADXL345_DATAX1          0x33
#define ADXL345_DATAY0          0x34
#define ADXL345_DATAY1          0x35
#define ADXL345_DATAZ0          0x36
#define ADXL345_DATAZ1          0x37
#define ADXL345_FIFO_CTL        0x38
#define ADXL345_FIFO_STATUS     0x39

// Address for L3G4200D
#define L3G4200D_WHO_AM_I       0x0F
#define L3G4200D_CTRL_REG1      0x20
#define L3G4200D_CTRL_REG2      0x21
#define L3G4200D_CTRL_REG3      0x22
#define L3G4200D_CTRL_REG4      0x23
#define L3G4200D_CTRL_REG5      0x24
#define L3G4200D_REFERENCE      0x25
#define L3G4200D_OUT_TEMP       0x26
#define L3G4200D_STATUS_REG     0x27
#define L3G4200D_OUT_X_L        0x28
#define L3G4200D_OUT_X_H        0x29
#define L3G4200D_OUT_Y_L        0x2A
#define L3G4200D_OUT_Y_H        0x2B
#define L3G4200D_OUT_Z_L        0x2C
#define L3G4200D_OUT_Z_H        0x2D
#define L3G4200D_FIFO_CTRL_REG  0x2E
#define L3G4200D_FIFO_SRC_REG   0x2F
#define L3G4200D_INT1_CFG       0x30
#define L3G4200D_INT1_SRC       0x31
#define L3G4200D_INT1_TSH_XH    0x32
#define L3G4200D_INT1_TSH_XL    0x33
#define L3G4200D_INT1_TSH_YH    0x34
#define L3G4200D_INT1_TSH_YL    0x35
#define L3G4200D_INT1_TSH_ZH    0x36
#define L3G4200D_INT1_TSH_ZL    0x37
#define L3G4200D_INT1_DURATION  0x38

// Address for MC5883L
#define HMC5883L_CONF_REG_A     0x00
#define HMC5883L_CONF_REG_B     0x01
#define HMC5883L_MODE_REG       0x02
#define HMC5883L_DATA_X_MSB     0x03
#define HMC5883L_DATA_X_LSB     0x04
#define HMC5883L_DATA_Y_MSB     0x05
#define HMC5883L_DATA_Y_LSB     0x06
#define HMC5883L_DATA_Z_MSB     0x07
#define HMC5883L_DATA_Z_LSB     0x08
#define HMC5883L_STAT_REG       0x09
#define HMC5883L_IDENT_REG_A    0x10
#define HMC5883L_IDENT_REG_B    0x11
#define HMC5883L_IDENT_REG_C    0x12

// Address for BMP085
#define BMP085_AC1              0xAA
#define BMP085_AC2              0xAC
#define BMP085_AC3              0xAE
#define BMP085_AC4              0xB0
#define BMP085_AC5              0xB2
#define BMP085_AC6              0xB4
#define BMP085_B1               0xB6
#define BMP085_B2               0xB8
#define BMP085_MB               0xBA
#define BMP085_MC               0xBC
#define BMP085_MD               0xBE

