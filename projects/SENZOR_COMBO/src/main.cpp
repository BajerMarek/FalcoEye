//if (rkButtonIsPressed(BTN_DOWN)) { // Je tlačítko dolů stisknuté?
#include <iostream>
#include <Arduino.h>
#include <thread>

#include "RBCX.h"
#include "uart_comand.h"
#include"i2c_tools.h"

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_VL53L0X.h"

// deklarace instanci senzoru
#define I2C_SLAVE_DEVICE_ADDRESS 0x8A  // Registr pro změnu adresy
#define VL53L0X_DEFAULT_ADDR 0x29  // Výchozí adresa senzoru
#define VL53L0X_NEW_ADDR     0x31  // Požadovaná nová adresa

#define XSHUT1 33
#define XSHUT2 13 //ten blize

Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
static const uint8_t TCS_SDA_pin = 14;
static const uint8_t TCS_SCL_pin = 26;

void cervena(){
  // here will be the color value -> red = 1, blue = 0

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
    //return true;
  }
  if (blue_avg > green_avg && blue_avg > red_avg && blue_avg > 100){
    //return false;
  }
}

void setup() {
  Serial.begin(115200);
  while (! Serial) {
    delay(1);
  }

  //Wire.begin(21, 22);


    Wire.begin(21, 22);  // SDA = GPIO21, SCL = GPIO22
    delay(10);
  Serial.println("Wire (I2C) inicializováno na SDA=21, SCL=22");
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);

    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    delay(20);

    digitalWrite(XSHUT2, HIGH);
    delay(10);

  
    Serial.println("zacina inicializace #############");
    if (!sensor2.begin()) {
      Serial.println("Nepodařilo se spustit senzor 2");
      while (1);
    }
    
    scan_i2c();
  //writeRegister(0x30,0x8A,0x31);
  sensor2.setAddress(0x31);
  scan_i2c();

  digitalWrite(XSHUT1, HIGH);
  delay(10);
  scan_i2c();

  if (!sensor1.begin()) {
    Serial.println("Nepodařilo se spustit senzor 1");
    while (1);
  }
  scan_i2c();
 //writeRegister(0x29,0x8A,0x30);
    //scan_i2c();
  sensor1.setAddress(0x30);

Serial.println("############# zacina inicializace #############");
//scan_i2c();    

  pinMode(TCS_SDA_pin, PULLUP);

  pinMode(TCS_SCL_pin, PULLUP);

  Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro predni senzor 


  // tcs.begin(0x29,&Wire1);
  // if (!tcs.begin(0x29,&Wire1)) {
  //   Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
  //   while (1);
  // } else {
  //   Serial.println("TCS34725 detekován.");
  // }
    scan_i2c();
}      

void loop()
{
    //cervena();
    Serial.print("######################\n");
    VL53L0X_RangingMeasurementData_t m1, m2;
    
    sensor1.rangingTest(&m1, false);
    sensor2.rangingTest(&m2, false);
    //   VL53L0X_RangingMeasurementData_t measure;
    //   Serial.print("Reading a measurement... ");
      //lox.rangingTest(&measure, true); 

    //   if (measure.RangeStatus != 4) { 
    //     Serial.print("Distance (cm): "); Serial.println(measure.RangeMilliMeter / 10.0);
    //   } else {
    //     Serial.println("Nothing in range! ");
    //   }
    
    Serial.print("Senzor 1 (0x30): ");
    Serial.print((m1.RangeStatus != 4)  ? String(m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    Serial.print("Senzor 2 (0x31): ");
    Serial.print((m2.RangeStatus != 4)  ? String(m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    delay(100);
    
}
    

    
 // I don't need loop, because i'm using while(true) in setup (it doesn't require to create global variables)x