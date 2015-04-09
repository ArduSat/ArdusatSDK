/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
/*
 * Modified by Ben Peters (Ardusat) to remove everything but constant definitions
 * for use in the Ardusat SDK package
 */

#ifndef __LSM303_H__
#define __LSM303_H__

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    //#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
    #define LSM303_ADDRESS_ACCEL          (0x1E >> 1)         // 0011001x
    #define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                                     // DEFAULT    TYPE
      LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
      LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
      LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
      LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
      LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
      LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
      LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
      LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
      LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
      LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
      LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
      LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
      LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
      LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
      LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
      LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
      LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
      LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
      LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
      LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
      LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
      LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
      LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
      LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
      LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
      LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
    } lsm303AccelRegisters_t;
    
    typedef enum
    {
      LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
      LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
      LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
      LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
      LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
      LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
      LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
      LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
      LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
      LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
      LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
      LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
      LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
      LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
      LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
    } lsm303MagRegisters_t;
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
      LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
      LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
      LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
      LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
      LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
      LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
    } lsm303MagGain;	
/*=========================================================================*/

/*
 * Defines for the LSM303 DTR (used on Lemsens)
 * These defines are from the pololu LSM driver:
 * https://github.com/pololu/lsm303-arduino
 */
enum deviceType { device_DLH,
                  device_DLM,
                  device_DLHC,
                  device_D,
                  device_auto
};

// register addresses
enum regAddr
{
  LSM303_TEMP_OUT_L        = 0x05, // D
  LSM303_TEMP_OUT_H        = 0x06, // D

  LSM303_STATUS_M          = 0x07, // D

  LSM303_INT_CTRL_M        = 0x12, // D
  LSM303_INT_SRC_M         = 0x13, // D
  LSM303_INT_THS_L_M       = 0x14, // D
  LSM303_INT_THS_H_M       = 0x15, // D

  LSM303_OFFSET_X_L_M      = 0x16, // D
  LSM303_OFFSET_X_H_M      = 0x17, // D
  LSM303_OFFSET_Y_L_M      = 0x18, // D
  LSM303_OFFSET_Y_H_M      = 0x19, // D
  LSM303_OFFSET_Z_L_M      = 0x1A, // D
  LSM303_OFFSET_Z_H_M      = 0x1B, // D
  LSM303_REFERENCE_X       = 0x1C, // D
  LSM303_REFERENCE_Y       = 0x1D, // D
  LSM303_REFERENCE_Z       = 0x1E, // D

  LSM303_CTRL0             = 0x1F, // D
  LSM303_CTRL1             = 0x20, // D
  LSM303_CTRL_REG1_A       = 0x20, // DLH, DLM, DLHC
  LSM303_CTRL2             = 0x21, // D
  LSM303_CTRL_REG2_A       = 0x21, // DLH, DLM, DLHC
  LSM303_CTRL3             = 0x22, // D
  LSM303_CTRL_REG3_A       = 0x22, // DLH, DLM, DLHC
  LSM303_CTRL4             = 0x23, // D
  LSM303_CTRL_REG4_A       = 0x23, // DLH, DLM, DLHC
  LSM303_CTRL5             = 0x24, // D
  LSM303_CTRL_REG5_A       = 0x24, // DLH, DLM, DLHC
  LSM303_CTRL6             = 0x25, // D
  LSM303_CTRL_REG6_A       = 0x25, // DLHC
  LSM303_HP_FILTER_RESET_A = 0x25, // DLH, DLM
  LSM303_CTRL7             = 0x26, // D
  LSM303_REFERENCE_A       = 0x26, // DLH, DLM, DLHC
  LSM303_STATUS_A          = 0x27, // D
  LSM303_STATUS_REG_A      = 0x27, // DLH, DLM, DLHC

  LSM303_OUT_X_L_A         = 0x28,
  LSM303_OUT_X_H_A         = 0x29,
  LSM303_OUT_Y_L_A         = 0x2A,
  LSM303_OUT_Y_H_A         = 0x2B,
  LSM303_OUT_Z_L_A         = 0x2C,
  LSM303_OUT_Z_H_A         = 0x2D,

  LSM303_FIFO_CTRL         = 0x2E, // D
  LSM303_FIFO_CTRL_REG_A   = 0x2E, // DLHC
  LSM303_FIFO_SRC          = 0x2F, // D
  LSM303_FIFO_SRC_REG_A    = 0x2F, // DLHC

  LSM303_IG_CFG1           = 0x30, // D
  LSM303_INT1_CFG_A        = 0x30, // DLH, DLM, DLHC
  LSM303_IG_SRC1           = 0x31, // D
  LSM303_INT1_SRC_A        = 0x31, // DLH, DLM, DLHC
  LSM303_IG_THS1           = 0x32, // D
  LSM303_INT1_THS_A        = 0x32, // DLH, DLM, DLHC
  LSM303_IG_DUR1           = 0x33, // D
  LSM303_INT1_DURATION_A   = 0x33, // DLH, DLM, DLHC
  LSM303_IG_CFG2           = 0x34, // D
  LSM303_INT2_CFG_A        = 0x34, // DLH, DLM, DLHC
  LSM303_IG_SRC2           = 0x35, // D
  LSM303_INT2_SRC_A        = 0x35, // DLH, DLM, DLHC
  LSM303_IG_THS2           = 0x36, // D
  LSM303_INT2_THS_A        = 0x36, // DLH, DLM, DLHC
  LSM303_IG_DUR2           = 0x37, // D
  LSM303_INT2_DURATION_A   = 0x37, // DLH, DLM, DLHC

  LSM303_CLICK_CFG         = 0x38, // D
  LSM303_CLICK_CFG_A       = 0x38, // DLHC
  LSM303_CLICK_SRC         = 0x39, // D
  LSM303_CLICK_SRC_A       = 0x39, // DLHC
  LSM303_CLICK_THS         = 0x3A, // D
  LSM303_CLICK_THS_A       = 0x3A, // DLHC
  LSM303_TIME_LIMIT        = 0x3B, // D
  LSM303_TIME_LIMIT_A      = 0x3B, // DLHC
  LSM303_TIME_LATENCY      = 0x3C, // D
  LSM303_TIME_LATENCY_A    = 0x3C, // DLHC
  LSM303_TIME_WINDOW       = 0x3D, // D
  LSM303_TIME_WINDOW_A     = 0x3D, // DLHC

  LSM303_Act_THS           = 0x3E, // D
  LSM303_Act_DUR           = 0x3F, // D

  LSM303_CRA_REG_M         = 0x00, // DLH, DLM, DLHC
  LSM303_CRB_REG_M         = 0x01, // DLH, DLM, DLHC
  LSM303_MR_REG_M          = 0x02, // DLH, DLM, DLHC

  LSM303_SR_REG_M          = 0x09, // DLH, DLM, DLHC
  LSM303_IRA_REG_M         = 0x0A, // DLH, DLM, DLHC
  LSM303_IRB_REG_M         = 0x0B, // DLH, DLM, DLHC
  LSM303_IRC_REG_M         = 0x0C, // DLH, DLM, DLHC

  LSM303_WHO_AM_I_M        = 0x0F, // DLM
  LSM303_WHO_AM_I          = 0x0F, // D

  LSM303_TEMP_OUT_H_M      = 0x31, // DLHC
  LSM303_TEMP_OUT_L_M      = 0x32, // DLHC


  // dummy addresses for registers in different locations on different devices;
  // the library translates these based on device type
  // value with sign flipped is used as index into translated_regs array

  LSM303_OUT_X_H_M         = -1,
  LSM303_OUT_X_L_M         = -2,
  LSM303_OUT_Y_H_M         = -3,
  LSM303_OUT_Y_L_M         = -4,
  LSM303_OUT_Z_H_M         = -5,
  LSM303_OUT_Z_L_M         = -6,
  // update dummy_reg_count if registers are added here!

  // device-specific register addresses

  LSM303_DLH_OUT_X_H_M     = 0x03,
  LSM303_DLH_OUT_X_L_M     = 0x04,
  LSM303_DLH_OUT_Y_H_M     = 0x05,
  LSM303_DLH_OUT_Y_L_M     = 0x06,
  LSM303_DLH_OUT_Z_H_M     = 0x07,
  LSM303_DLH_OUT_Z_L_M     = 0x08,

  LSM303_DLM_OUT_X_H_M     = 0x03,
  LSM303_DLM_OUT_X_L_M     = 0x04,
  LSM303_DLM_OUT_Z_H_M     = 0x05,
  LSM303_DLM_OUT_Z_L_M     = 0x06,
  LSM303_DLM_OUT_Y_H_M     = 0x07,
  LSM303_DLM_OUT_Y_L_M     = 0x08,

  LSM303_DLHC_OUT_X_H_M    = 0x03,
  LSM303_DLHC_OUT_X_L_M    = 0x04,
  LSM303_DLHC_OUT_Z_H_M    = 0x05,
  LSM303_DLHC_OUT_Z_L_M    = 0x06,
  LSM303_DLHC_OUT_Y_H_M    = 0x07,
  LSM303_DLHC_OUT_Y_L_M    = 0x08,

  LSM303_D_OUT_X_L_M       = 0x08,
  LSM303_D_OUT_X_H_M       = 0x09,
  LSM303_D_OUT_Y_L_M       = 0x0A,
  LSM303_D_OUT_Y_H_M       = 0x0B,
  LSM303_D_OUT_Z_L_M       = 0x0C,
  LSM303_D_OUT_Z_H_M       = 0x0D
};


static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

/*=========================================================================
    CHIP ID
    -----------------------------------------------------------------------*/
    #define LSM303_ID                     (0b11010100)
/*=========================================================================*/

#endif
