#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// here will be the color value -> red = 1, blue = 0
bool barvy(){
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
      //printf("RED\n");
      return true;
    }
    if (blue_avg > green_avg && blue_avg > red_avg && blue_avg > 100){
      return false;
    }
}

// vrátí vzdálenost v milimetrech
void vzdalenost()
   {
     VL53L0X_RangingMeasurementData_t measure;
     Serial.print("Reading a measurement... ");
     lox.rangingTest(&measure, true); 
     
     if (measure.RangeStatus != 4) { 
       Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter );
     } else {
       Serial.println("Nothing in range! ");
     }
     delay(100); 
}