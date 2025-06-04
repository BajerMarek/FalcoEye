#include "robotka.h"
#include "RBCX.h"
void setup() {
    Serial.begin(115200);
    rkConfig cfg;
    rkSetup(cfg);

    // Nastavení serva
    rkServosSetPosition(1, 0); // Servo 1 nastaví na 90°
    delay(1000);

    rkServosSetPosition(1, 90); // Servo 1 nastaví na 0°
    delay(1000);

    rkServosSetPosition(1, 0); // Servo 1 nastaví na 90°
    delay(1000);
    //rkServosSetPosition(1, ); // Servo 1 nastaví na 180°
    //delay(1000);
}

void loop() {
    // Serva zůstávají na poslední pozici
}