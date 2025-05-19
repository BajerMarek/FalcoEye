#include <Arduino.h>
#include<uart_comand.h>
// //! skore čím větší tím lepší
// // Předpokládáme, že MAX_OD_BOX_CNT je nějaký definovaný počet (např. 10)
// #define MAX_OD_BOX_CNT 20

// // Definice struktury ODResult_t
// typedef struct tagODResult_t {
//     union {
//         int16_t xyxy[4];  // Při použití jako array
//         struct {
//             int16_t x1;
//             int16_t y1;
//             int16_t x2;
//             int16_t y2;  // Při použití jako jednotlivé hodnoty
//         };
//     };
//     int16_t score10;  // Skóre * 10
//     int16_t color;	// here will be the color value -> red = 1, blue = 0
// } ODResult_t;

// // Definice struktury UARTResult_t
// typedef struct tagUARTResult_t {
//     uint8_t header;          // Hlavička
//     uint8_t leng;            // Délka dat
//     int16_t suma;            // Součet
//     ODResult_t results_array[MAX_OD_BOX_CNT];  // Pole výsledků
// } UARTResult_t;

void setup(){
    uart_set_up();
    //Serial.begin(115200);
}

void loop() {
    // Serial.print("ahoj");
    // delay(200);
    UARTResult_t vstup;
   
    if (uart_recive(vstup)) {
        Serial.printf("Header: %d, Length: %d, Sum: %d\n",
            vstup.header, vstup.leng, vstup.suma);
        //Serial.print("####### datarecieved #######\n");
        //Mám teď celý packet, můžu pracovat
        for (int i = 0; i < vstup.leng; i++) {

                Serial.printf("x1: %d, y1: %d, x2: %d, y2: %d score: %d, color: %d\n",
                            vstup.results_array[i].x1,
                            vstup.results_array[i].y1,
                            vstup.results_array[i].x2,
                            vstup.results_array[i].y2,
                            vstup.results_array[i].score10,
                            vstup.results_array[i].color);
        }
    }
    //delay(100);
}
