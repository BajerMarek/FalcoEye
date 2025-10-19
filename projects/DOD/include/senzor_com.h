#include <iostream>
#include <Arduino.h>

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_VL53L0X.h"


// převod na metr encoder 4000 cca -> na jednom metru jsem naměřil 4000 encoderu
typedef struct tagI2cout
{
    // barevný senzor

    float r;
    float g;
    float b;

    //laseový senzor

    VL53L0X_RangingMeasurementData_t m1;
    VL53L0X_RangingMeasurementData_t m2;

}I2cout;

void cervena(Adafruit_TCS34725 tcs){
    // potřeba inicializace tsc -> Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
  // here will be the color value -> red = 1, blue = 0
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
    //return true;
  }
  if (blue_avg > green_avg && blue_avg > red_avg && blue_avg > 100){
    //return false;
  }
}

 // sensor2 -> třída pro dálkový senzor
 // sensor1 -> třída pro dálkový senzor
 // tcs -> třída pro dálkový senzor
 // TCS_SDA_pin -> SDA pin pro barevný senzor -defaultně 14
 // TCS_SCL_pin -> SCL pin pro barevný senzor - defaultně 26
 // senzor_data -> vpíše data do struktury která bude dána jako parametr
// void get_senzor_data(Adafruit_VL53L0X sensor2, 
//     Adafruit_VL53L0X sensor1,    
//     Adafruit_TCS34725 tcs,       
//      int TCS_SDA_pin = 14,       
//      int TCS_SCL_pin = 26,       
//      I2cout &senzor_data)  
// {
//     while(1)
//     {
//     pinMode(TCS_SDA_pin, PULLUP);

//     pinMode(TCS_SCL_pin, PULLUP);

//     Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro predni senzor 


//     //tcs.begin(0x29,&Wire1);
//     if (!tcs.begin(0x29,&Wire1)) {
//         Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
//         while (1);
//     } else {
//         //Serial.println("TCS34725 detekován.");
//     }

//     tcs.getRGB(&senzor_data.r,&senzor_data.g,&senzor_data.b);

//     // dalkové senzory
//      sensor1.rangingTest(&senzor_data.m1, false);
//      sensor2.rangingTest(&senzor_data.m2, false);
//      delay(10);

//     }

// }