#include <Wire.h>
#include <Arduino.h>

//! funkčnízmění adreus senzoru
void writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

//! funkčí najde zařízení na zběrnici
void scan_i2c()
{
  Serial.println("Scanning I2C bus...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at address: 0x");
      Serial.println(addr, HEX);
    }
  }
}