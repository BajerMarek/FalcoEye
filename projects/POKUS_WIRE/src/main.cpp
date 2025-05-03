#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 16, 17, false, 20000000UL ); // trida HardwareSerial (speed, config, Rx, Tx, invert, timeout )
}

int i = 0;
const int length_of_struct = 10;
u_int8_t struct_bytes[length_of_struct]; 
// u_int16_t 

void loop() {
    // delay(500);
    // Serial.printf(" %d  ", i++);
    //Serial.write(15);
    if(Serial1.available() ) {

        Serial1.readBytes(struct_bytes, length_of_struct);
        // Serial.write(struct_bytes, length_of_struct); 
        for (size_t i = 0; i < length_of_struct; i++)
        {
            Serial.printf("%d  ", struct_bytes[i]);
        }
        
        Serial.println(";");
    }
}