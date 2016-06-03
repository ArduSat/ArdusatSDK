#ifndef LSM303_h
#define LSM303_h

#include <Arduino.h> // for byte data type

/*
 * Modified by Sam Olds (Ardusat) to move enums out of class and into #defines
 */

#define TEMP_OUT_L         0x05 // D
#define TEMP_OUT_H         0x06 // D

#define STATUS_M           0x07 // D

#define INT_CTRL_M         0x12 // D
#define INT_SRC_M          0x13 // D
#define INT_THS_L_M        0x14 // D
#define INT_THS_H_M        0x15 // D

#define OFFSET_X_L_M       0x16 // D
#define OFFSET_X_H_M       0x17 // D
#define OFFSET_Y_L_M       0x18 // D
#define OFFSET_Y_H_M       0x19 // D
#define OFFSET_Z_L_M       0x1A // D
#define OFFSET_Z_H_M       0x1B // D
#define REFERENCE_X        0x1C // D
#define REFERENCE_Y        0x1D // D
#define REFERENCE_Z        0x1E // D

#define CTRL0              0x1F // D
#define CTRL1              0x20 // D
#define CTRL_REG1_A        0x20 // DLH, DLM, DLHC
#define CTRL2              0x21 // D
#define CTRL_REG2_A        0x21 // DLH, DLM, DLHC
#define CTRL3              0x22 // D
#define CTRL_REG3_A        0x22 // DLH, DLM, DLHC
#define CTRL4              0x23 // D
#define CTRL_REG4_A        0x23 // DLH, DLM, DLHC
#define CTRL5              0x24 // D
#define CTRL_REG5_A        0x24 // DLH, DLM, DLHC
#define CTRL6              0x25 // D
#define CTRL_REG6_A        0x25 // DLHC
#define HP_FILTER_RESET_A  0x25 // DLH, DLM
#define CTRL7              0x26 // D
#define REFERENCE_A        0x26 // DLH, DLM, DLHC
#define STATUS_A           0x27 // D
#define STATUS_REG_A       0x27 // DLH, DLM, DLHC

#define OUT_X_L_A          0x28
#define OUT_X_H_A          0x29
#define OUT_Y_L_A          0x2A
#define OUT_Y_H_A          0x2B
#define OUT_Z_L_A          0x2C
#define OUT_Z_H_A          0x2D

#define FIFO_CTRL          0x2E // D
#define FIFO_CTRL_REG_A    0x2E // DLHC
#define FIFO_SRC           0x2F // D
#define FIFO_SRC_REG_A     0x2F // DLHC

#define IG_CFG1            0x30 // D
#define INT1_CFG_A         0x30 // DLH, DLM, DLHC
#define IG_SRC1            0x31 // D
#define INT1_SRC_A         0x31 // DLH, DLM, DLHC
#define IG_THS1            0x32 // D
#define INT1_THS_A         0x32 // DLH, DLM, DLHC
#define IG_DUR1            0x33 // D
#define INT1_DURATION_A    0x33 // DLH, DLM, DLHC
#define IG_CFG2            0x34 // D
#define INT2_CFG_A         0x34 // DLH, DLM, DLHC
#define IG_SRC2            0x35 // D
#define INT2_SRC_A         0x35 // DLH, DLM, DLHC
#define IG_THS2            0x36 // D
#define INT2_THS_A         0x36 // DLH, DLM, DLHC
#define IG_DUR2            0x37 // D
#define INT2_DURATION_A    0x37 // DLH, DLM, DLHC

#define CLICK_CFG          0x38 // D
#define CLICK_CFG_A        0x38 // DLHC
#define CLICK_SRC          0x39 // D
#define CLICK_SRC_A        0x39 // DLHC
#define CLICK_THS          0x3A // D
#define CLICK_THS_A        0x3A // DLHC
#define TIME_LIMIT         0x3B // D
#define TIME_LIMIT_A       0x3B // DLHC
#define TIME_LATENCY       0x3C // D
#define TIME_LATENCY_A     0x3C // DLHC
#define TIME_WINDOW        0x3D // D
#define TIME_WINDOW_A      0x3D // DLHC

#define Act_THS            0x3E // D
#define Act_DUR            0x3F // D

#define CRA_REG_M          0x00 // DLH, DLM, DLHC
#define CRB_REG_M          0x01 // DLH, DLM, DLHC
#define MR_REG_M           0x02 // DLH, DLM, DLHC

#define SR_REG_M           0x09 // DLH, DLM, DLHC
#define IRA_REG_M          0x0A // DLH, DLM, DLHC
#define IRB_REG_M          0x0B // DLH, DLM, DLHC
#define IRC_REG_M          0x0C // DLH, DLM, DLHC

#define WHO_AM_I           0x0F // D
#define WHO_AM_I_M         0x0F // DLM

#define TEMP_OUT_H_M       0x31 // DLHC
#define TEMP_OUT_L_M       0x32 // DLHC


// dummy addresses for registers in different locations on different devices;
// the library translates these based on device type
// value with sign flipped is used as index into translated_regs array

#define OUT_X_H_M          -1
#define OUT_X_L_M          -2
#define OUT_Y_H_M          -3
#define OUT_Y_L_M          -4
#define OUT_Z_H_M          -5
#define OUT_Z_L_M          -6
// update dummy_reg_count if registers are added here!

// device-specific register addresses

#define DLH_OUT_X_H_M      0x03
#define DLH_OUT_X_L_M      0x04
#define DLH_OUT_Y_H_M      0x05
#define DLH_OUT_Y_L_M      0x06
#define DLH_OUT_Z_H_M      0x07
#define DLH_OUT_Z_L_M      0x08

#define DLM_OUT_X_H_M      0x03
#define DLM_OUT_X_L_M      0x04
#define DLM_OUT_Z_H_M      0x05
#define DLM_OUT_Z_L_M      0x06
#define DLM_OUT_Y_H_M      0x07
#define DLM_OUT_Y_L_M      0x08

#define DLHC_OUT_X_H_M     0x03
#define DLHC_OUT_X_L_M     0x04
#define DLHC_OUT_Z_H_M     0x05
#define DLHC_OUT_Z_L_M     0x06
#define DLHC_OUT_Y_H_M     0x07
#define DLHC_OUT_Y_L_M     0x08

#define D_OUT_X_L_M        0x08
#define D_OUT_X_H_M        0x09
#define D_OUT_Y_L_M        0x0A
#define D_OUT_Y_H_M        0x0B
#define D_OUT_Z_L_M        0x0C
#define D_OUT_Z_H_M        0x0D

class LSM303
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    enum deviceType { device_DLH, device_DLM, device_DLHC, device_D, device_auto };
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    vector<int16_t> a; // accelerometer readings
    vector<int16_t> m; // magnetometer readings

    byte last_status; // status of last I2C transmission

    LSM303(void);

    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    deviceType getDeviceType(void) { return _device; }

    void enableDefault(void);

    void writeAccReg(byte reg, byte value);
    byte readAccReg(byte reg);
    void writeMagReg(byte reg, byte value);
    byte readMagReg(int reg);

    void writeReg(byte reg, byte value);
    byte readReg(int reg);

    void readAcc(void);
    void readMag(void);
    void read(void);

  private:
    deviceType _device; // chip type (D, DLHC, DLM, or DLH)
    byte acc_address;
    byte mag_address;

    static const int dummy_reg_count = 6;
    uint8_t translated_regs[dummy_reg_count + 1]; // index 0 not used

    unsigned int io_timeout;
    bool did_timeout;

    int testReg(byte address, uint8_t reg);
};

#endif
