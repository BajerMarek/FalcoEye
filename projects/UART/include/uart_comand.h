#include <Arduino.h>
#include<tuple>

// max počet detekovaných objektů
#define MAX_OD_BOX_CNT 20

// Definice přijímané struktury 
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

// Definice struktury UARTResult_t
typedef struct tagUARTResult_t {
    uint8_t header;          // Hlavička
    uint8_t leng;            // Délka dat
    int16_t suma;            // Součet
    ODResult_t results_array[MAX_OD_BOX_CNT];  // Pole výsledků
} UARTResult_t;
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

//získá data posílaná přes uart z nxp
//výstup: [x1, y1, x2, y2, score, color] -> souřadnice protilehlých rohů bouding boxu, score = procentualní přesnost *10, barva najitého objektu
std::array<ODResult_t, MAX_OD_BOX_CNT>get_uart_data()
{
// array s výsledky
std::array<ODResult_t, MAX_OD_BOX_CNT> results;

// Pokud máme nějaká data na UART2
if (Serial2.available()) {
    // Načteme jeden byte do bufferu
    struct_bytes[bytesReceived] = Serial2.read();
    bytesReceived++;
    
    // Pokud máme celou strukturu
    if (bytesReceived >= length_of_struct) {
        // Deserializace přijatých dat na strukturu UARTResult_t
        UARTResult_t receivedPacket;
        memcpy(&receivedPacket, struct_bytes, sizeof(UARTResult_t));
            // Vypíšeme přijaté hodnoty na hlavní sériový port
            Serial.printf("Received UARTResult_t:\n");
            Serial.printf("Header: %d\n", receivedPacket.header);
            Serial.printf("Length: %d\n", receivedPacket.leng);
            Serial.printf("Suma: %d\n", receivedPacket.suma);
            
            // Vypíšeme výsledky
            for (int i = 0; i < MAX_OD_BOX_CNT; i++) {
                Serial.printf("Result %d: (x1: %d, y1: %d, x2: %d, y2: %d, score10: %d, color: %d)\n",
                              i,
                              receivedPacket.results_array[i].x1,
                              receivedPacket.results_array[i].y1,
                              receivedPacket.results_array[i].x2,
                              receivedPacket.results_array[i].y2,
                              receivedPacket.results_array[i].score10,
                            receivedPacket.results_array[i].color);
            }
            // Resetujeme byte count pro příjem dalších dat
            bytesReceived = 0;
        }
    }
    return results;
}






