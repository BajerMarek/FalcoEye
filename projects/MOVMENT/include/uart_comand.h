#include <Arduino.h>

#include <cstdint>

uint16_t slozit_bajty(uint8_t byte1, uint8_t byte2) {
    return (static_cast<uint16_t>(byte2) << 8) | byte1;
}

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
    int16_t score10;  // procentualní přesnost *10
    int16_t color;	// here will be the color value -> red = 1, blue = 0
    int16_t name;   // number of the object to recognize it from others
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
        Serial1.begin(115200, SERIAL_8N1, 16,17); // Nastavení UART2
        while (!Serial) {
            ; // Čekáme na připojení sériového portu (pokud používáš USB)
        }
        Serial.println("Serial started, waiting for data...");
}
// Definujeme velikost struktury UARTResult_t

const int length_of_struct = sizeof(ODResult_t);
uint16_t struct_bytes[length_of_struct];

size_t bytesReceived = 0;  // Počet přijatých bajtů
bool uart_recive(UARTResult_t &output)
{
    if (Serial1.available()<4) return false;

    uint8_t hlavicka = Serial1.read();
    if(hlavicka==255)
    {
        uint8_t pocet_puku = Serial1.read();
        if((pocet_puku<20)&&(pocet_puku>0))
        {
            uint8_t crc_suma = Serial1.read();
            uint8_t crc_suma2 = Serial1.read();
            crc_suma = slozit_bajty(crc_suma,crc_suma2);
            //Serial.printf("Hlavicka: %d | pocet puku: %d | crc: %d\n", hlavicka,pocet_puku,crc_suma);
            output.header = hlavicka;
            output.leng = pocet_puku;
            output.suma = crc_suma;
    //! po sem vše funguje
if (Serial1.available() < sizeof(ODResult_t) * pocet_puku)      // počet bajtů pro příjmutí
        return false;

    // Čtení přímo do output.results_array
    uint16_t* p = (uint16_t*)output.results_array;
    for (int i = 0; i < (sizeof(ODResult_t)/2) * pocet_puku; i++)    // pocet int16_t bajtu v posílané struktuře
    {
        uint8_t low = Serial1.read();
        uint8_t high = Serial1.read();
        p[i] = slozit_bajty(low, high);
    }
                //bytesReceived = 0;  // připraven na další packet
            return true;             
        }
    }
    return false;
}
// vezme proměnou a uloží do ní strukturu UARTResult_t + vrátí true jestli je celý
bool get_uart_data(UARTResult_t &output) {
    while (Serial1.available() && bytesReceived < length_of_struct) {
        struct_bytes[bytesReceived++] = Serial1.read();
    }
    if (bytesReceived >= length_of_struct) {
        memcpy(&output, struct_bytes, sizeof(tagUARTResult_t));
        // Debug print
        Serial.printf("Header: %d, Length: %d, Sum: %d\n",
                      output.header, output.leng, output.suma);
        bytesReceived = 0;  // připraven na další packet
        return true;
    }
    return false;
}
