#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
    Serial.begin(115200);

    while (! Serial) {
        delay(1);
    }

    Serial.println("VL53L0X test");
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        while(1);
    }
    
}

void loop() {

 VL53L0X_RangingMeasurementData_t measure;
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, true); 

  if (measure.RangeStatus != 4) { 
    Serial.print("Distance (cm): "); Serial.println(measure.RangeMilliMeter / 10.0);
  } else {
    Serial.println("Nothing in range! ");
  }
  
  delay(1000);
}