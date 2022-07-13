/**
 * @file NAU7802.cpp
 * @author Phil Hilger (phil.hilger@waterlooti.com)
 * @brief
 * @version 0.1
 * @date 2022-05-17
 *
 * (c) Copyright 2022, Waterloo Tech Inc.
 *
 */

#include "NAU7802.h"
#include "esp_log.h"

static const char *TAG = "NAU7802";

NAU7802::NAU7802(i2c_port_t i2cPort, uint8_t i2cAddress, gpio_num_t intPin)
    : i2cPort(i2cPort), i2cAddress(i2cAddress), intPin(intPin) {}

NAU7802::~NAU7802() {}

bool NAU7802::init() {
  /* Check for NAU7802 revision register (0x1F), low nibble should be 0xF. */
  uint8_t value;
  if (!enable(true)) {
    ESP_LOGE(TAG, "Can't wakeup sensor");
  }
  if (!reset()) {
    ESP_LOGE(TAG, "Can't reset sensor");
    return false;
  }
  if (!enable(true)) {
    ESP_LOGE(TAG, "Can't enable sensor");
    return false;
  }
  if (!readRegister(NAU7802_REVISION_ID, &value) || (value & 0xF) != 0xF) {
    ESP_LOGE(TAG, "Wrong sensor");
    return false;
  }
  if (!setLDO(NAU7802_3V0)) {
    ESP_LOGE(TAG, "Can't set sensor voltage");
    return false;
  }
  if (!setGain(NAU7802_GAIN_128)) {
    ESP_LOGE(TAG, "Can't set sensor gain");
    return false;
  }
  if (!setRate(NAU7802_RATE_40SPS)) {
    ESP_LOGE(TAG, "Can't set sensor rate");
    return false;
  }

  // disable ADC chopper clock
  if (!readRegister(NAU7802_ADC, &value) ||
      !writeRegister(NAU7802_ADC, setBits(value, 2, 4, 0x03))) {
    return false;
  };

  // use low ESR caps
  if (!readRegister(NAU7802_PGA, &value) ||
      !writeRegister(NAU7802_PGA, setBits(value, 1, 6, 0x00))) {
    return false;
  };

  // PGA stabilizer cap on output
  if (!readRegister(NAU7802_POWER, &value) ||
      !writeRegister(NAU7802_POWER, setBits(value, 1, 7, 0x01))) {
    return false;
  };

  return true;
}

/**************************************************************************/
/*!
    @brief  Whether to have the sensor enabled and working or in power down mode
    @param  flag True to be in powered mode, False for power down mode
    @return False if something went wrong with I2C comms
*/
/**************************************************************************/
bool NAU7802::enable(bool flag) {
  uint8_t value = 0;
  if (!readRegister(NAU7802_PU_CTRL, &value) && !flag) {
    ESP_LOGE(TAG, "Enable - NAU7802_PU_CTRL = %02x", value);
    return false;
  }

  if (!flag) {
    return writeRegister(NAU7802_PU_CTRL,
                         setBits(value, 2, 1, 0x00)); // analog=0, digital=0
  }

  // turn on!
  if (!writeRegister(NAU7802_PU_CTRL,
                     setBits(value, 2, 1, 0x03))) { // analog=1, digital=1
    return false;
  }

  // RDY: Analog part wakeup stable plus Data Ready after exiting power-down
  // mode 600ms
  vTaskDelay(pdMS_TO_TICKS(600));
  // re-read and update value
  if (!readRegister(NAU7802_PU_CTRL, &value) ||
      !writeRegister(NAU7802_PU_CTRL, setBits(value, 1, 4, 0x01))) {
    return false;
  }
  return readRegister(NAU7802_PU_CTRL, &value) &&
         (value & (1 << 3)); // ready==1
}

/**************************************************************************/
/*!
    @brief Whether there is new ADC data to read
    @return True when there's new data available
*/
/**************************************************************************/
bool NAU7802::available(void) {
  uint8_t value;
  return (readRegister(NAU7802_PU_CTRL, &value) &&
          (value & (1<<5))); // available==1
}

/**************************************************************************/
/*!
    @brief Read the stored 24-bit ADC output value.
    @return Signed integer with ADC output result, extended to a int32_t
*/
/**************************************************************************/
int32_t NAU7802::read(void) {
  uint32_t value = 0;

  readADC(&value);
  // extend sign bit
  if (value & 0x800000) {
    value |= 0xFF000000;
  }

  return value;
}

/**************************************************************************/
/*!
    @brief Perform a soft reset
    @return False if there was any I2C comms error
*/
/**************************************************************************/
bool NAU7802::reset(void) {
  uint8_t value;
  if (!readRegister(NAU7802_PU_CTRL, &value)) {
    return false;
  }

  // Set the RR bit to 1 in R0x00, to guarantee a reset of all register values.
  if (!writeRegister(NAU7802_PU_CTRL, setBits(value, 1, 0, 0x01))) { // reset=1
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(10));
  if (!writeRegister(NAU7802_PU_CTRL,
                     setBits(value, 2, 0, 0x02))) { // reset=0, digital=1
    return false;
  }
  // After about 200 microseconds, the PWRUP bit will be Logic=1 indicating the
  // device is ready for the remaining programming setup.
  vTaskDelay(pdMS_TO_TICKS(1));
  return (readRegister(NAU7802_PU_CTRL, &value) &&
          (value & (1 << 3))); // ready==1
}

/**************************************************************************/
/*!
    @brief  The desired LDO voltage setter
    @param voltage The LDO setting: NAU7802_4V5, NAU7802_4V2, NAU7802_3V9,
    NAU7802_3V6, NAU7802_3V3, NAU7802_3V0, NAU7802_2V7, NAU7802_2V4, or
    NAU7802_EXTERNAL if we are not using the internal LDO
    @return False if there was any I2C comms error
*/
/**************************************************************************/
bool NAU7802::setLDO(NAU7802_LDOVoltage voltage) {
  uint8_t value;
  if (!readRegister(NAU7802_PU_CTRL, &value)) {
    return false;
  }

  if (voltage == NAU7802_EXTERNAL) {
    // special case!
    return writeRegister(NAU7802_PU_CTRL,
                         setBits(value, 1, 7, 0x00)); // avdds=0
  }

  // internal LDO
  if (!writeRegister(NAU7802_PU_CTRL, setBits(value, 1, 7, 0x01))) { // avdds=1
    return false;
  }
  return writeRegister(NAU7802_CTRL1,
                       setBits(value, 3, 3, (uint8_t)voltage)); // vldo=voltage
}

/**************************************************************************/
/*!
    @brief  The desired LDO voltage getter
    @returns The voltage setting: NAU7802_4V5, NAU7802_4V2, NAU7802_3V9,
    NAU7802_3V6, NAU7802_3V3, NAU7802_3V0, NAU7802_2V7, NAU7802_2V4, or
    NAU7802_EXTERNAL if we are not using the internal LDO
*/
/**************************************************************************/
NAU7802_LDOVoltage NAU7802::getLDO(void) {
  uint8_t value;
  if (!readRegister(NAU7802_PU_CTRL, &value) || ((value & (1 << 7)) == 0)) {
    return NAU7802_EXTERNAL;
  }

  // internal LDO
  if (!readRegister(NAU7802_CTRL1, &value)) {
    return NAU7802_EXTERNAL;
  }
  return (NAU7802_LDOVoltage)((value >> 3) & 0x07);
}

/**************************************************************************/
/*!
    @brief  The desired ADC gain setter
    @param  gain Desired gain: NAU7802_GAIN_1, NAU7802_GAIN_2, NAU7802_GAIN_4,
    NAU7802_GAIN_8, NAU7802_GAIN_16, NAU7802_GAIN_32, NAU7802_GAIN_64,
    or NAU7802_GAIN_128
    @returns False if there was any error during I2C comms
*/
/**************************************************************************/
bool NAU7802::setGain(NAU7802_Gain gain) {
  uint8_t value;
  return (readRegister(NAU7802_CTRL1, &value) &&
          writeRegister(NAU7802_CTRL1, setBits(value, 3, 0, (uint8_t)gain)));
}

/**************************************************************************/
/*!
    @brief  The desired ADC gain getter
    @returns The gain: NAU7802_GAIN_1, NAU7802_GAIN_2, NAU7802_GAIN_4,
    NAU7802_GAIN_8, NAU7802_GAIN_16, NAU7802_GAIN_32, NAU7802_GAIN_64,
    or NAU7802_GAIN_128
*/
/**************************************************************************/
NAU7802_Gain NAU7802::getGain(void) {
  uint8_t value = 0;
  readRegister(NAU7802_CTRL1, &value);
  return (NAU7802_Gain)(value & 0x07);
}

/**************************************************************************/
/*!
    @brief  The desired conversion rate setter
    @param rate The desired rate: NAU7802_RATE_10SPS, NAU7802_RATE_20SPS,
    NAU7802_RATE_40SPS, NAU7802_RATE_80SPS, or NAU7802_RATE_320SPS
    @returns False if any I2C error occured
*/
/**************************************************************************/
bool NAU7802::setRate(NAU7802_SampleRate rate) {
  uint8_t value;
  return (readRegister(NAU7802_CTRL2, &value) &&
          writeRegister(NAU7802_CTRL2, setBits(value, 3, 4, (uint8_t)rate)));
}

/**************************************************************************/
/*!
    @brief  The desired conversion rate getter
    @returns The rate: NAU7802_RATE_10SPS, NAU7802_RATE_20SPS,
    NAU7802_RATE_40SPS, NAU7802_RATE_80SPS, or NAU7802_RATE_320SPS
*/
/**************************************************************************/
NAU7802_SampleRate NAU7802::getRate(void) {
  uint8_t value = 0;
  readRegister(NAU7802_CTRL2, &value);
  return (NAU7802_SampleRate)((value >> 4) & 0x07);
}

/**************************************************************************/
/*!
    @brief  Perform the internal calibration procedure
    @param mode The calibration mode to perform: NAU7802_CALMOD_INTERNAL,
    NAU7802_CALMOD_OFFSET or NAU7802_CALMOD_GAIN
    @returns True on calibrations success
*/
/**************************************************************************/
bool NAU7802::calibrate(NAU7802_Calibration mode) {
  uint8_t value;
  if (!readRegister(NAU7802_CTRL2, &value)) {
    return false;
  }
  if (!writeRegister(NAU7802_CTRL2,
                     setBits(value, 2, 0, mode))) { // calibration mode
    return false;
  }
  if (!readRegister(NAU7802_CTRL2, &value) ||
      writeRegister(NAU7802_CTRL2,
                    setBits(value, 1, 2, 0x01))) { // start calibration
    return false;
  }
  while (!readRegister(NAU7802_CTRL2, &value) ||
         ((value & (1 << 2)) == 0)) { // wait for start
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return readRegister(NAU7802_CTRL2, &value) &&
         ((value & (1 << 3)) == 0); // wait for start
}

/**
 * @brief read a register into value byte and return true on success
 *
 * @param regAddress
 * @param value
 * @return true
 * @return false
 */
bool NAU7802::readRegister(uint8_t regAddress, uint8_t *value) {
  *value = 0;
  esp_err_t err = i2c_master_write_read_device(i2cPort, i2cAddress, &regAddress,
                                               1, value, 1, I2C_WAIT_MS);
  ESP_LOGD(TAG, "Read 0x%02x = 0x%02x (err = %d)", regAddress, *value, err);
  return (ESP_OK == err);
}

/**
 * @brief write byte value into register and return true on success
 *
 * @param regAddress
 * @param value
 * @return true
 * @return false
 */
bool NAU7802::writeRegister(uint8_t regAddress, uint8_t value) {
  uint8_t tx[2] = {regAddress, value};
  esp_err_t err =
      i2c_master_write_to_device(i2cPort, i2cAddress, tx, 2, I2C_WAIT_MS);
  ESP_LOGD(TAG, "Write 0x%02x <- 0x%02x (err = %d)", regAddress, value, err);

  return (ESP_OK == err);
}

/**
 * @brief read ADC value on 24 bits and return true on sucess
 *
 * @param regAddress
 * @param value
 * @return true
 * @return false
 */
bool NAU7802::readADC(uint32_t *value) {
  uint8_t buffer[3];
  uint8_t regAddress = NAU7802_ADCO_B2;
  esp_err_t err =
      i2c_master_write_read_device(i2cPort, i2cAddress, &regAddress, 1, buffer, 3, I2C_WAIT_MS);
  if (ESP_OK != err) {
    ESP_LOGE(TAG, "ReadADC err = %d", err);
    return false;
  }
  *value = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2]);
  ESP_LOGD(TAG, "ReadADC = %u", *value);
  return true;
}

/**
 * @brief change bits bits in value
 *
 * @param value initial value
 * @param bits number of bits to change
 * @param bitShift number of bits to shift
 * @param bitValue new value for bits
 */
uint8_t NAU7802::setBits(uint8_t value, int bits, int bitShift,
                         uint8_t bitValue) {
  return ((value & ~(((1 << bits) - 1) << bitShift))) | (bitValue << bitShift);
}