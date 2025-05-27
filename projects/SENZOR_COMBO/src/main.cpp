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

void cervena(){
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
//#include "robotka.h"

// std::mutex uart_mutex;  //! zamek pro komunikaci s vláknem -> pokud otevřeno poze jedno vlákno může komunikovat jinak nic
//UARTResult_t latest_uart_data;  //? globalni proměná pro udládání dat z uartu
UARTResult_t uart_data;
I2cout senzor_data;
// Funkce pro vlákno, které čte data z UARTu
// void uart_thread_function() {
//     while (true) {
//         UARTResult_t uart_data;
//         if (uart_recive(uart_data)) { // Funkce pro příjem dat z UARTu
//             std::lock_guard<std::mutex> lock(uart_mutex); //! Zamkneme mutex pro další používání + odemkne při skončení skopu
//             memcpy(&latest_uart_data,&uart_data,sizeof (uart_data));
//             //latest_uart_data = uart_data; // Uložíme data
//         }//? teď se mutex odemkne pro datlší používání
        
//         std::this_thread::sleep_for(sd::chrono::milliseconds(10)); // Snížení zatížení CPU -> uspí vlákno na 10ms
//     }
// }

void get_data(UARTResult_t vstup)
{
    while(1)
    {
      //  if(uart_recive(vstup))
      //  {
      //      memcpy(&uart_data,&vstup,sizeof (vstup));

      //  }

        delay(10);
    }
}
void get_senzor_data()
{
    while(1)
    {
    pinMode(TCS_SDA_pin, PULLUP);

    pinMode(TCS_SCL_pin, PULLUP);

    Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro predni senzor 


    //tcs.begin(0x29,&Wire1);
    if (!tcs.begin(0x29,&Wire1)) {
        Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
        while (1);
    } else {
        //Serial.println("TCS34725 detekován.");
    }


    //  float r, g, b;
    //  tcs.getRGB(&r, &g, &b);
    //  senzor_data.r = r;
    //  senzor_data.g = g;
    //  senzor_data.b = b;
    //  delay(10);
    tcs.getRGB(&senzor_data.r,&senzor_data.g,&senzor_data.b);

    // // dalkové senzory
    //  sensor1.rangingTest(&senzor_data.m1, false);
    //  sensor2.rangingTest(&senzor_data.m2, false);
    //  delay(10);

    }

}
void rampa(int target, int a)
{

}
void encoder(int cas)
{
    //m1 musí být -
    int M1_pos, M4_pos, odhylaka, integral, last_odchylka =0;
    int target = 20000;
    int a = 500;
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    for(int i = 0; i < target-1; i+=a)
    {
        odhylaka = M1_pos-M4_pos;
        integral += odhylaka; 
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        man.motor(rb::MotorId::M1).power(-i);
        man.motor(rb::MotorId::M4).power(i+odhylaka*20);
        delay(10);
        
    }
    integral =0;    //###
    for(int i =0; i<cas;i+=50)
    {
        odhylaka = M1_pos-M4_pos;
        integral += odhylaka; 

        man.motor(rb::MotorId::M1).power(-20000);
        man.motor(rb::MotorId::M4).power(20000+odhylaka*55+integral*0.01+(odhylaka-last_odchylka)*0.1);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        std::cout<<"odchylak: "<<M1_pos-M4_pos<<std::endl;

        delay(50);
        last_odchylka = odhylaka;
    }
    for(int i = 0; i > target+1; i-=a)
    {
        odhylaka = M1_pos-M4_pos;
        integral += odhylaka; 
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        man.motor(rb::MotorId::M1).power(-i);
        man.motor(rb::MotorId::M4).power(i+odhylaka*20);
        delay(10);
        
    }

}

void tocka(int smer, int cas)   // smer 1 -> pravo -1 -> levo
{
    //m1 musí být -
    //int M1_pos, M4_pos, odhylaka, integral, last_odchylka =0;
    //int target = 10000;
    //int a = 500;
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    
    //for(int i =0; i<cas;i+=50)
    //{
        man.motor(rb::MotorId::M1).power(20000*smer);
        man.motor(rb::MotorId::M4).power(20000*smer);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            //M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            //M4_pos = info.position();
        });
        //std::cout<<"odchylak: "<<M1_pos-M4_pos<<std::endl;

        delay(cas);

    //}
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

}
void kupredu(int M1_dis, int M4_dis)
{
    //m1 musí být -
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M4).power(16000);

}
void pokus()
{
    for(int i = 0; i<3; i++) {
        micros(); // update overflow
        kupredu(16000,16000);
        printf("lmotor power: %d rmotor power: %d\n", 32767, 32767);
        delay(1000);
        kupredu(-16000,-16000);
        delay(1000);
        

        rb::Manager::get()
            .setMotors()
            .power(rb::MotorId::M1, 0)
            .power(rb::MotorId::M4, 0)
            .set();
        printf("lmotor power: %d rmotor power: %d\n", 0, 0);
        delay(5000);
    }
}


void setup() {
  Serial.begin(115200);
  while (! Serial) {
    delay(1);
  }

  Wire.begin(21, 22);
//   // delay(10);
//   // Serial.println("Wire (I2C) inicializováno na SDA=21, SCL=22");

//     //Wire.begin(21, 22);  // SDA = GPIO21, SCL = GPIO22
//     pinMode(XSHUT1, OUTPUT);
//     pinMode(XSHUT2, OUTPUT);

//     digitalWrite(XSHUT1, LOW);
//     digitalWrite(XSHUT2, LOW);
//     delay(20);

//     digitalWrite(XSHUT2, HIGH);
//     delay(10);

  
//     Serial.println("zacina inicializace #############");
//     if (!sensor2.begin()) {
//       Serial.println("Nepodařilo se spustit senzor 2");
//       while (1);
//     }
    
//     //scan_i2c();
//   //writeRegister(0x30,0x8A,0x31);
//   sensor2.setAddress(0x31);
//   scan_i2c();

//   digitalWrite(XSHUT1, HIGH);
//   delay(10);
//   scan_i2c();

//   if (!sensor1.begin()) {
//     Serial.println("Nepodařilo se spustit senzor 1");
//     while (1);
//   }
//   scan_i2c();
//  // writeRegister(0x29,0x8A,0x30);
//     scan_i2c();
//   sensor1.setAddress(0x30);

//     Serial.println("############# zacina inicializace #############");
//     //scan_i2c();    

  // pinMode(TCS_SDA_pin, PULLUP);

  // pinMode(TCS_SCL_pin, PULLUP);

  // Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro predni senzor 


  // //tcs.begin(0x29,&Wire1);
  // if (!tcs.begin(0x29,&Wire1)) {
  //   Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
  //   while (1);
  // } else {
  //   Serial.println("TCS34725 detekován.");
  // }

    // printf("RB3204-RBCX\n");
    // delay(50);
    // printf("Init manager\n");

    // auto& man = rb::Manager::get(); // get manager instance as singleton
    // man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    // micros(); // update overflow
    // delay(200);

    // uart_set_up();


    // //delay(1000);


    // //lox.begin();
    // delay(200);

    // writeRegister(VL53L0X_DEFAULT_ADDR, I2C_SLAVE_DEVICE_ADDRESS, VL53L0X_NEW_ADDR);
    // delay(200);
    
    // Inicializace senzoru na nové adrese

    Serial.println("Senzor VL53L0X uspesne inicializovan.");
    tcs.begin(0x29);

    //uart_thread_function();
    // std::thread uart_thread (get_data, uart_data);
    // uart_thread.detach();

    std::thread i2c_thread(get_senzor_data);
    i2c_thread.detach();
    Serial.println("start");
    // while (true) { // drive motor M1 to position 1000 with 100% power (32767) if button down is pressed and return to 0 position if button is released
    //     if (man.buttons().down()) {
        //         man.leds().green(true);
        //         man.setMotors().driveToValue(rb::MotorId::M1, 1000, 32767).set();
        //     } else {
            //         man.leds().green(false);
            //         man.setMotors().driveToValue(rb::MotorId::M1, 0, 32767).set();
            //     }
            // }
            // UARTResult_t uart_data_uart;
            
            
            //! puvodni program
            // if (man.buttons().down()) {
                //     printf("Tlacitko stisknuto\n");
                
                
                //     encoder(3000);
                
                //     // //     printf("Tlacitko stisknuto\n");
                //     // //     //delay(1000);
                //     // //     man.motor(rb::MotorId::M1).power(-16000);
                //     // //     man.motor(rb::MotorId::M4).power(16000);
                //     // //     delay(3000);
                //     // //     // man.setMotors().driveToValue(rb::MotorId::M1, 100*ticksToMm, -32767).set();
                //     // //     // man.setMotors().driveToValue(rb::MotorId::M4, 100*ticksToMm, 32767).set();
                //     // } else {
                    //     //     man.motor(rb::MotorId::M1).brake(16000);
                    //     //     man.motor(rb::MotorId::M4).brake(16000);
                    //     //     delay(3000);
                    //     // }
                    //     //!tady konci
                    // }
                    
                    
                    
                    // // delay(1000);
                    // // kupredu(16000,16000);
                    // // delay(10000);
                    // // if (rkButtonIsPressed(BTN_DOWN)) { // Je tlačítko dolů stisknuté?
                    // //     printf("***************START******************\n");
                    // //     delay(1000);
                    // //     kupredu(16000,16000);
                    // //     delay(3000);
                    // // }
                    
                }
                
void loop()
{
//     //cervena();
    Serial.printf("red: %f, green: %f, blue: %f",senzor_data.r,senzor_data.g,senzor_data.b);
    Serial.print("######################\n");
//     // VL53L0X_RangingMeasurementData_t m1, m2;
    
//     // sensor1.rangingTest(&m1, false);
//     // sensor2.rangingTest(&m2, false);
//     //   VL53L0X_RangingMeasurementData_t measure;
//     //   Serial.print("Reading a measurement... ");
//     //   lox.rangingTest(&measure, true); 

//     //   if (measure.RangeStatus != 4) { 
//     //     Serial.print("Distance (cm): "); Serial.println(measure.RangeMilliMeter / 10.0);
//     //   } else {
//     //     Serial.println("Nothing in range! ");
//     //   }
    
//     Serial.print("Senzor 1 (0x30): ");
//     Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
//     Serial.print("\n");
//     Serial.print("Senzor 2 (0x31): ");
//     Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
//     Serial.print("\n");
//     delay(100);
//     //Serial.println("ping");
//     //std::lock_guard<std::mutex> lock(uart_mutex);
//     //Serial.println(uart_data.header);
//     Serial.println("---- UART START ----");
    
//     //Serial.printf("BARVA JE: %d",uart_data.results_array->color);
//     int x1 =0;
//     int x2 =0;
//     int center = 125;
// //  Serial.printf("Header: %d, Length: %d, Sum: %d\n",
// //             uart_data.header, uart_data.leng, uart_data.suma);
// //             leg = uart_data.leng;
//     if(uart_data.header == 255)
//     {
//         Serial.printf("Header: %d, Length: %d, Sum: %d\n",
//             uart_data.header, uart_data.leng, uart_data.suma);
            
//         //Serial.print("####### datarecieved #######\n");
//         //Mám teď celý packet, můžu pracovat
//         for (int i = 0; i < uart_data.leng; i++) {

//                 Serial.printf("x1: %d, y1: %d, x2: %d, y2: %d score: %d, color: %d, name: %d\n",
//                             uart_data.results_array[i].x1,
//                             uart_data.results_array[i].y1,
//                             uart_data.results_array[i].x2,
//                             uart_data.results_array[i].y2,
//                             uart_data.results_array[i].score10,
//                             uart_data.results_array[i].color,
//                         uart_data.results_array[i].name);
//                 x1 =uart_data.results_array[i].x1;     // 30 min
//                 x2 =uart_data.results_array[i].x2;     // 280 max

//                 if(uart_data.results_array[i].color&&(!(uart_data.results_array[i].name)))
//                 {
//                     int object_center = x1+((x2 - x1) / 2); //! Výpočet středu objektu do samostatné proměnné pro lepší čitelnost
//                     Serial.printf("center je: %d",object_center);
//                     if (object_center < 185 && object_center > 110) { //! Upraveno pro použití object_center
//                         Serial.println("##### MAM TO #####");
//                     }
//                     else if (object_center > center) { //! Upraveno pro použití object_center
//                                 Serial.println("##### RRRRRRRRRRRRRRRRRRRRRRRRR #####");
//                         tocka(-1, 100);
//                         //delay(300);
//                     }
//                     else if (object_center < center && object_center > 30) { //! Upraveno pro použití object_center
//                         Serial.println("##### LLLLLLLLLLLLLLLLLLL #####");
//                         tocka(1, 100);
//                         //delay(300);
//                     }
//                 }
//                 else{
//                     Serial.println("------------- je to modre -------------");
    //             }
    //     }
    // }
    
    
    
}
    

    
 // I don't need loop, because i'm using while(true) in setup (it doesn't require to create global variables)x