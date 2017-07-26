/*************************************************** 
  This is a library for the Adafruit DRV2605L Haptic Driver

  ----> http://www.adafruit.com/products/2306
 
  Check out the links above for our tutorials and wiring diagrams
  This motor/haptic driver uses I2C to communicate

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include <Adafruit_DRV2605.h>

/**************************************************************************/
/*! 
    @brief  Instantiates a new DRV2605 class
*/
/**************************************************************************/
// I2C, no address adjustments or pins
Adafruit_DRV2605::Adafruit_DRV2605() {
  _actuatorType = ACTUATOR_TYPE_ERM;
}

Adafruit_DRV2605::Adafruit_DRV2605(uint8_t actuatorType) {
  _actuatorType = actuatorType;
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
boolean Adafruit_DRV2605::begin() {
  Wire.begin();
  uint8_t id = getDeviceId();
  /*
  Serial.print("ID: ");
  Serial.println(id, DEC);
  */
  if ((id != DEVICE_ID_DRV2605) && (id != DEVICE_ID_DRV2604) &&
      (id != DEVICE_ID_DRV2604L) && (id != DEVICE_ID_DRV2605L)) {
    return false;
  }

  // device reset
  reset();
  writeRegister8(DRV2605_REG_MODE, 0x00); // out of standby
  
  writeRegister8(DRV2605_REG_RTPIN, 0x00); // no real-time-playback
  
  writeRegister8(DRV2605_REG_WAVESEQ1, 1); // strong click
  writeRegister8(DRV2605_REG_WAVESEQ2, 0);
  
  writeRegister8(DRV2605_REG_OVERDRIVE, 0); // no overdrive
  
  writeRegister8(DRV2605_REG_SUSTAINPOS, 0);
  writeRegister8(DRV2605_REG_SUSTAINNEG, 0);
  writeRegister8(DRV2605_REG_BREAK, 0);
  writeRegister8(DRV2605_REG_AUDIOMAX, 0x64);
  
  switch (_actuatorType) {
    case ACTUATOR_TYPE_LRA:
      // turn on N_ERM_LRA (LRA actuator)
      useLRA();

      // auto-resonance mode enabled
      writeRegister8(DRV2605_REG_CONTROL3, readRegister8(DRV2605_REG_CONTROL3) & ~(0x01));
      break;

    case ACTUATOR_TYPE_ERM:
    default:
      // ERM open loop
      // turn off N_ERM_LRA (ERM actuator)
      useERM();

      // turn on ERM_OPEN_LOOP
      writeRegister8(DRV2605_REG_CONTROL3, readRegister8(DRV2605_REG_CONTROL3) | 0x20);
      break;
  }

  return true;
}

void Adafruit_DRV2605::setWaveform(uint8_t slot, uint8_t w) {
  writeRegister8(DRV2605_REG_WAVESEQ1+slot, w);
}

void Adafruit_DRV2605::selectLibrary(uint8_t lib) {
  writeRegister8(DRV2605_REG_LIBRARY, lib);
}

void Adafruit_DRV2605::reset() {
  // device reset - write 1 to DEV_RESET field
  writeRegister8(DRV2605_REG_MODE, 0x80);
}

void Adafruit_DRV2605::go() {
  writeRegister8(DRV2605_REG_GO, 1);
}

void Adafruit_DRV2605::stop() {
  writeRegister8(DRV2605_REG_GO, 0);
}

void Adafruit_DRV2605::setMode(uint8_t mode) {
  writeRegister8(DRV2605_REG_MODE, mode);
}

void Adafruit_DRV2605::setRealtimeValue(uint8_t rtp) {
  writeRegister8(DRV2605_REG_RTPIN, rtp);
}

uint8_t Adafruit_DRV2605::getDeviceId() {
  return ((readRegister8(DRV2605_REG_STATUS) & 0xE0) >> 5);
}

/********************************************************************/

uint8_t Adafruit_DRV2605::readRegister8(uint8_t reg) {
  uint8_t x ;
   // use i2c
    Wire.beginTransmission(DRV2605_ADDR);
    Wire.write((byte)reg);
    Wire.endTransmission();
    Wire.requestFrom((byte)DRV2605_ADDR, (byte)1);
    x = Wire.read();

  //  Serial.print("$"); Serial.print(reg, HEX); 
  //  Serial.print(": 0x"); Serial.println(x, HEX);
  
  return x;
}

void Adafruit_DRV2605::writeRegister8(uint8_t reg, uint8_t val) {
   // use i2c
    Wire.beginTransmission(DRV2605_ADDR);
    Wire.write((byte)reg);
    Wire.write((byte)val);
    Wire.endTransmission();
}

/****************/


// Allow users to use ERM motor or LRA motors

void Adafruit_DRV2605::useERM ()
{
  writeRegister8(DRV2605_REG_FEEDBACK, readRegister8(DRV2605_REG_FEEDBACK) & 0x7F);
}

void Adafruit_DRV2605::useLRA ()
{
  writeRegister8(DRV2605_REG_FEEDBACK, readRegister8(DRV2605_REG_FEEDBACK) | 0x80);
}



