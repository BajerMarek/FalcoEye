#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//! zmena adresy dalkového senzoru (VL53LOX)
#define VL53L0X_DEFAULT_ADDR 0x29  // Výchozí adresa senzoru
#define VL53L0X_NEW_ADDR     0x31  // Požadovaná nová adresa
#define I2C_SLAVE_DEVICE_ADDRESS 0x8A  // Registr pro změnu adresy


/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */
   
   /* Initialise with default values (int time = 2.4ms, gain = 1x) */
   // Adafruit_TCS34725 tcs = Adafruit_TCS34725();
   /* Initialise with specific int time and gain values */

void vzdalenost()
   {

     VL53L0X_RangingMeasurementData_t measure;
     Serial.print("Reading a measurement... ");
     lox.rangingTest(&measure, true); 
     
     if (measure.RangeStatus != 4) { 
       Serial.print("Distance (cm): "); Serial.println(measure.RangeMilliMeter / 10.0);
     } else {
       Serial.println("Nothing in range! ");
     }
     delay(100); 
   }
   
bool cervena(){
  static const uint8_t TCS_SDA_pin = 21;
  static const uint8_t TCS_SCL_pin = 22;
  pinMode(TCS_SDA_pin, PULLUP);
  pinMode(TCS_SCL_pin, PULLUP);
  Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000);
  float r, g, b;
  tcs.getRGB(&r, &g, &b);
  float red_avg = r;
  float green_avg = g;
  float blue_avg = b;
  printf("red: %f, green: %f, blue: %f\n", red_avg, green_avg, blue_avg);
  delay(10);
  if (red_avg > green_avg && red_avg > blue_avg && red_avg > 100)
  {
    printf("RED\n");
    
    return true;
  }
  else {
    return false;
  }
}

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

void setup()
{
  Serial.begin(115200);

  lox.begin();
  delay(200);

  writeRegister(VL53L0X_DEFAULT_ADDR, I2C_SLAVE_DEVICE_ADDRESS, VL53L0X_NEW_ADDR);
  delay(200);
  // 2️⃣ Po změně adresy můžeš pokračovat komunikací na nové adrese
  // Inicializace senzoru na nové adrese
  if (!lox.begin(VL53L0X_NEW_ADDR)) {
      Serial.println("Chyba: Senzor VL53L0X nebyl nalezen!");
      while (1);
  }
  Serial.println("Senzor VL53L0X úspěšně inicializován.");
  tcs.begin(0x29);

  while(true)
  {

    cervena();
    Serial.println("*****************************************");
    vzdalenost();
  }
}
void loop()
{

}