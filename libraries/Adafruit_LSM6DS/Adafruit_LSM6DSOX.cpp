
/*!
 *  @file Adafruit_LSM6DSOX.cpp
 *  Adafruit LSM6DSOX 6-DoF Accelerometer and Gyroscope library
 *
 *  Bryan Siepert for Adafruit Industries
 * 	BSD (see license.txt)
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_LSM6DSOX.h"

/*!
 *    @brief  Instantiates a new LSM6DSOX class
 */
Adafruit_LSM6DSOX::Adafruit_LSM6DSOX(void) {}

bool Adafruit_LSM6DSOX::_init(int32_t sensor_id) {
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_WHOAMI);

  // make sure we're talking to the right chip
  if (chip_id.read() != LSM6DSOX_CHIP_ID) {
    return false;
  }
  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;

  reset();

  // Block Data Update
  // this prevents MSB/LSB data registers from being updated until both are read
  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL3_C);
  Adafruit_BusIO_RegisterBits bdu = Adafruit_BusIO_RegisterBits(&ctrl3, 1, 6);
  bdu.write(true);

  // Disable I3C
  Adafruit_BusIO_Register ctrl_9 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL9_XL);
  Adafruit_BusIO_RegisterBits i3c_disable_bit =
      Adafruit_BusIO_RegisterBits(&ctrl_9, 1, 1);

  i3c_disable_bit.write(true);

  // call base class _init()
  Adafruit_LSM6DS::_init(sensor_id);

  return true;
}

/**************************************************************************/
/*!
    @brief Disables and enables the SPI master bus pulllups.
    @param disable_pullups true to **disable** the I2C pullups, false to enable.
*/
void Adafruit_LSM6DSOX::disableSPIMasterPullups(bool disable_pullups) {

  Adafruit_BusIO_Register pin_config = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_PIN_CTRL);

  Adafruit_BusIO_RegisterBits disable_ois_pu =
      Adafruit_BusIO_RegisterBits(&pin_config, 1, 7);

  disable_ois_pu.write(disable_pullups);
}

/**************************************************************************/
/*!
    @brief Enables and disables the I2C master bus pulllups.
    @param enable_pullups true to enable the I2C pullups, false to disable.
*/
void Adafruit_LSM6DSOX::enableI2CMasterPullups(bool enable_pullups) {

  Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_FUNC_CFG_ACCESS);
  Adafruit_BusIO_RegisterBits master_cfg_enable_bit =
      Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 6);

  Adafruit_BusIO_Register master_config = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_MASTER_CONFIG);
  Adafruit_BusIO_RegisterBits i2c_master_pu_en =
      Adafruit_BusIO_RegisterBits(&master_config, 1, 3);

  master_cfg_enable_bit.write(true);
  i2c_master_pu_en.write(enable_pullups);
  master_cfg_enable_bit.write(false);
}


void Adafruit_LSM6DSOX::SOXenablePedometer(bool enable) {
	
  Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_FUNC_CFG_ACCESS);
	  
  Adafruit_BusIO_Register page_rw = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_PAGE_RW);  
  
  Adafruit_BusIO_Register page_sel = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_PAGE_SEL);
	  
  Adafruit_BusIO_Register page_address = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_PAGE_ADDRESS);

  Adafruit_BusIO_Register page_value = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_PAGE_VALUE);	
	  
  Adafruit_BusIO_Register emb_func_en_a = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_EMB_FUNC_EN_A);
  
  Adafruit_BusIO_Register emb_func_en_b = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_EMB_FUNC_EN_B);

  Adafruit_BusIO_Register emb_func_int1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_EMB_FUNC_INT1);
	  
  Adafruit_BusIO_Register md1_cfg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_MD1_CFG);
	  
  Adafruit_BusIO_Register ctrl1_xl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL1_XL);

  Adafruit_BusIO_Register pedo_cmd_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_PEDO_CMD_REG);

  Adafruit_BusIO_Register emb_func_src = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_EMB_FUNC_SRC);
	  
  Adafruit_BusIO_Register emb_func_init_a = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_EMB_FUNC_INIT_A);
	  

  
  //ctrl1_xl.write(0x28);
  //md1_cfg.write(0x02);
  //func_cfg_access.write(0x80);
  //page_rw.write(0x40);
  //page_sel.write(0x11);
  //page_address.write(0x83);
  //page_value.write(0x04);
  //page_rw.write(0x00);
  //emb_func_en_a.write(0x08);
  //emb_func_en_b.write(0x10);
  //emb_func_int1.write(0x08);
  //func_cfg_access.write(0x00);
  
  
  ctrl1_xl.write(0x30);
  md1_cfg.write(0x02);
  func_cfg_access.write(0x80);
  emb_func_src.write(0x80);
  emb_func_en_a.write(0x08);
  emb_func_int1.write(0x08);
  emb_func_init_a.write(0x08);
  
  
  
  //page_rw.write(0x40);
  //page_sel.write(0x11);
  //page_address.write(0x83);
  //page_value.write(0x04);
  //page_rw.write(0x00);
  
  //emb_func_en_b.write(0x10);
  
  func_cfg_access.write(0x00);
  
  

}

uint16_t Adafruit_LSM6DSOX::SOXreadPedometer() {
  Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_FUNC_CFG_ACCESS);
	  
  
	  
  func_cfg_access.write(0x80);
  
  Adafruit_BusIO_Register steps_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, 0x62, 2);
  uint16_t steps = steps_reg.read();
  
  func_cfg_access.write(0x00);
	  
  return steps;
}

