#include <Arduino.h>

// max počet detekovaných objektů
#define MAX_OD_BOX_CNT 20

// Definice struktury přijímaných dat 
typedef struct tagODResult_t {
    union {
        int16_t xyxy[4];  // Při použití jako array
        struct {
            int16_t x1;
            int16_t y1;
            int16_t x2;
            int16_t y2;  // Při použití jako jednotlivé hodnoty
        };
    };
    int16_t score10;  // Skóre * 10
    int16_t color;	// here will be the color value -> red = 1, blue = 0
} ODResult_t;

// Definice struktury odesílaných dat UARTResult_t
typedef struct tagUARTResult_t {
    uint8_t header;          // Hlavička
    uint8_t leng;            // Délka dat
    int16_t suma;            // Součet
    ODResult_t results_array[MAX_OD_BOX_CNT];  // Pole výsledků
} UARTResult_t;

//nastavý uart
void uart_set_up(){
        Serial.begin(115200); // Hlavní UART pro komunikaci s PC
        Serial2.begin(115200, SERIAL_8N1, 16,17); // Nastavení UART2
        while (!Serial) {
            ; // Čekáme na připojení sériového portu (pokud používáš USB)
        }
        Serial.println("Serial started, waiting for data...");
}
// Definujeme velikost struktury UARTResult_t
const int length_of_struct = sizeof(UARTResult_t);  // Velikost celé struktury UARTResult_t
uint8_t struct_bytes[length_of_struct];  // Buffer pro přijatá data
size_t bytesReceived = 0;  // Počet přijatých bajtů

// vezme proměnou a uloží do ní strukturu UARTResult_t + vrátí true jestli je celý
bool get_uart_data(UARTResult_t &output) {
    while (Serial2.available() && bytesReceived < length_of_struct) {
        struct_bytes[bytesReceived++] = Serial2.read();
    }
    if (bytesReceived >= length_of_struct) {
        memcpy(&output, struct_bytes, sizeof(UARTResult_t));
        // Debug print
        Serial.printf("Header: %d, Length: %d, Sum: %d\n",
                      output.header, output.leng, output.suma);
        bytesReceived = 0;  // připraven na další packet
        return true;
    }
    return false;
}
