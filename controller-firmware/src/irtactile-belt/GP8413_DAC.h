#ifndef GP8413_DAC_H
#define GP8413_DAC_H

#include <Arduino.h>   // map()

#include "driver/i2c.h"

#define I2C_MASTER_NUM  I2C_NUM_0
#define I2C_MASTER_SDA  21
#define I2C_MASTER_SCL  22
// 1 MHz, above the 400 kHz the GP8413 is specified for: it keeps the 5-byte
// sample write (~56 us) inside the DAC tick. See the tick-budget note in
// Playout.h before changing it.
#define I2C_MASTER_FREQ 1000000
#define I2C_TIMEOUT_MS  1

#define GP8413_ADDRESS 0x58


#define GP8413_REG_CONFIG 0x01
#define GP8413_REG_CH0    0x02
#define GP8413_REG_CH1    0x04
#define GP8413_REG_STORE  0x06

// Raw register values. Both nonzero, so they must be passed as bytes, never as
// a bool.
#define OUTPUT_RANGE_5V   0x55
#define OUTPUT_RANGE_10V  0x77


inline bool i2cMasterInit() {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ;

  if (i2c_param_config(I2C_MASTER_NUM, &conf) != ESP_OK) return false;
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0) == ESP_OK;
}


// Takes the register value itself (OUTPUT_RANGE_5V / OUTPUT_RANGE_10V).
inline bool writeConfig(uint8_t rangeValue) {
    uint8_t data[2];
    data[0] = GP8413_REG_CONFIG;
    data[1] = rangeValue;

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        GP8413_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(10)
    );

    return (err == ESP_OK);
}


// One GP8413 on this bus: every transaction addresses GP8413_ADDRESS directly,
// so the class takes no address.
class GP8413_DAC {
private:
    uint8_t _outputRange;

public:
    GP8413_DAC() : _outputRange(OUTPUT_RANGE_5V) {}
    
    bool begin(uint8_t outputRange = OUTPUT_RANGE_5V) {

        if (!i2cMasterInit()) {
            return false;
        }
        // The range write doubles as the presence probe: it is the first
        // transaction on the bus and only succeeds if the DAC ACKs.
        return setOutputRange(outputRange);
    }

    // Presence probe: re-writes the range currently in force, so it never
    // changes it.
    bool isConnected() {
        return writeConfig(_outputRange);

    }

    bool setOutputRange(uint8_t range) {
        _outputRange=range;
        return writeConfig(range);

    }
    
    bool setVoltage(uint16_t millivolts, uint8_t channel) {
        if (channel > 1) return false;
        
        uint16_t maxVoltage = (_outputRange == OUTPUT_RANGE_10V) ? 10000 : 5000;
        if (millivolts > maxVoltage) millivolts = maxVoltage;
        
        uint16_t dacValue = map(millivolts, 0, maxVoltage, 0, 32767);
        
        return setDACValue(dacValue, channel);
    }
    
bool setDACValue(uint16_t value, uint8_t channel) {
    if (channel > 1) return false;
    if (value > 32767) value = 32767; // 15-bit max

    uint8_t reg = (channel == 0) ? GP8413_REG_CH0 : GP8413_REG_CH1;

    uint8_t data[3];
    data[0] = reg;
    data[1] = value & 0xFF;        // lower 8 bits
    data[2] = (value >> 8) & 0x7F; // upper 7 bits

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        GP8413_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );

    return (err == ESP_OK);
    }

    // Both channels in one transaction. Not an overload of setDACValue:
    // (uint16_t, uint8_t) and (uint16_t, uint16_t) make any call with an integer
    // literal second argument ambiguous.
    //
    // The GP8413 auto-increments its register pointer, so one write starting at
    // GP8413_REG_CH0 covers CH0 (0x02, 0x03) and CH1 (0x04, 0x05).
bool setBothDACValues(uint16_t value1, uint16_t value2) {

    if (value1 > 32767) value1 = 32767; // 15-bit max
    if (value2 > 32767) value2 = 32767; // 15-bit max

    uint8_t data[5];
    data[0] = GP8413_REG_CH0;       // auto-increments from here
    data[1] = value1 & 0xFF;        // CH0 lower 8 bits
    data[2] = (value1 >> 8) & 0x7F; // CH0 upper 7 bits
    data[3] = value2 & 0xFF;        // CH1 lower 8 bits
    data[4] = (value2 >> 8) & 0x7F; // CH1 upper 7 bits

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        GP8413_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );

    return (err == ESP_OK);
    }

    bool setBothChannels(uint16_t ch0Value, uint16_t ch1Value) {

        return setBothDACValues(ch0Value, ch1Value) ;

    }
    
    bool setBothVoltages(uint16_t ch0_mv, uint16_t ch1_mv) {

        uint16_t maxVoltage = (_outputRange == OUTPUT_RANGE_10V) ? 10000 : 5000;
        if (ch0_mv > maxVoltage) ch0_mv = maxVoltage;
        if (ch1_mv > maxVoltage) ch1_mv = maxVoltage;

return setBothChannels(map(ch0_mv, 0, maxVoltage, 0, 32767), map(ch1_mv, 0, maxVoltage, 0, 32767));

    }
    
    uint16_t getMaxVoltage() {
        return (_outputRange == OUTPUT_RANGE_10V) ? 10000 : 5000;
    }
    
    uint8_t getOutputRange() {
        return _outputRange;
    }
};

#endif