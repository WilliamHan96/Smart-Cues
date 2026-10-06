/*!
 *  @file Adafruit_LSM6DSOX.h
 *
 * 	I2C Driver for the Adafruit LSM6DSOX 6-DoF Accelerometer and Gyroscope
 *library
 *
 * 	This is a library for the Adafruit LSM6DSOX breakout:
 * 	https://www.adafruit.com/products/PID_HERE
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSOX_H
#define _ADAFRUIT_LSM6DSOX_H

#include "Adafruit_LSM6DS.h"

#define LSM6DSOX_CHIP_ID 0x6C ///< LSM6DSOX default device id from WHOAMI

#define LSM6DSOX_FUNC_CFG_ACCESS 0x1 ///< Enable embedded functions register
#define LSM6DSOX_PIN_CTRL 0x2        ///< Pin control register

#define LSM6DSOX_INT1_CTRL 0x0D ///< Interrupt enable for data ready
#define LSM6DSOX_CTRL1_XL 0x10  ///< Main accelerometer config register
#define LSM6DSOX_CTRL2_G 0x11   ///< Main gyro config register
#define LSM6DSOX_CTRL3_C 0x12   ///< Main configuration register
#define LSM6DSOX_CTRL9_XL 0x18  ///< Includes i3c disable bit
#define LSM6DSOX_MD1_CFG 0x5e  ///< Includes i3c disable bit


#define LSM6DSOX_MASTER_CONFIG 0x14
///< I2C Master config; access must be enabled with  bit SHUB_REG_ACCESS
///< is set to '1' in FUNC_CFG_ACCESS (01h).

#define LSM6DSOX_EMB_FUNC_EN_A 0x04 ///<Enables embedded functions register (R/W)
#define LSM6DSOX_EMB_FUNC_EN_B 0x05 ///<Enable embedded functions register (R/W)
#define LSM6DSOX_EMB_FUNC_FIFO_CFG 0x44 ///<Embedded functions batching configuration register (R/W)
#define LSM6DSOX_EMB_FUNC_INT1 0x0A ///<INT1 pin control register (R/W)
#define LSM6DSOX_EMB_FUNC_INT2 0x0E ///<INT2 pin control register (R/W)
#define LSM6DSOX_EMB_FUNC_SRC 0x64 ///<Embedded function source register (R/W)
#define LSM6DSOX_EMB_FUNC_INIT_A 0x66 ///<Embedded functions initialization register (R/W)
#define LSM6DSOX_PEDO_CMD_REG 0x83 ///<Pedometer configuration register (R/W)
#define LSM6DSOX_PEDO_DEB_STEPS_CONF 0x84 ///<Pedometer debounce configuration register (R/W)
#define LSM6DSOX_PAGE_RW 0x17
#define LSM6DSOX_PAGE_SEL 0x02
#define LSM6DSOX_PAGE_ADDRESS 0x08
#define LSM6DSOX_PAGE_VALUE 0x09

#define LSM6DSOX_FIFO_CTRL4 0x0A ///<FIFO control register 4 (R/W)

class Adafruit_LSM6DSOX : public Adafruit_LSM6DS {
public:
  Adafruit_LSM6DSOX();

  void enableI2CMasterPullups(bool enable_pullups);
  void disableSPIMasterPullups(bool disable_pullups);
  void SOXenablePedometer(bool enable);
  void SOXresetPedometer();
  uint16_t SOXreadPedometer();

private:
  bool _init(int32_t sensor_id);
};

#endif
