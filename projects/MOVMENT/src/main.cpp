//if (rkButtonIsPressed(BTN_DOWN)) { // Je tlačítko dolů stisknuté?
#include <iostream>
#include <Arduino.h>
#include <thread>
#include <mutex>
#include "RBCX.h"
#include "uart_comand.h"
//#include "robotka.h"

// std::mutex uart_mutex;  //! zamek pro komunikaci s vláknem -> pokud otevřeno poze jedno vlákno může komunikovat jinak nic
//UARTResult_t latest_uart_data;  //? globalni proměná pro udládání dat z uartu
UARTResult_t uart_data;

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
       if( uart_recive(vstup))
       {
           memcpy(&uart_data,&vstup,sizeof (vstup));

       }
        delay(10);
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
    printf("RB3204-RBCX\n");
    delay(50);

    printf("Init manager\n");
    
    
    auto& man = rb::Manager::get(); // get manager instance as singleton
    man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    micros(); // update overflow
    delay(200);
    uart_set_up();
    //uart_thread_function();
    std::thread uart_thread (get_data, uart_data);
    uart_thread.detach();
    //delay(1000);
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
    //Serial.println("ping");
    //std::lock_guard<std::mutex> lock(uart_mutex);
    //Serial.println(uart_data.header);
    
    
    //Serial.printf("BARVA JE: %d",uart_data.results_array->color);
    int x1 =0;
    int x2 =0;
    int leg =0;
    int center = 125;
 Serial.printf("Header: %d, Length: %d, Sum: %d\n",
            uart_data.header, uart_data.leng, uart_data.suma);
            leg = uart_data.leng;
    if(uart_data.header == 255)
    {
        Serial.printf("Header: %d, Length: %d, Sum: %d\n",
            uart_data.header, uart_data.leng, uart_data.suma);
            leg = uart_data.leng;
        //Serial.print("####### datarecieved #######\n");
        //Mám teď celý packet, můžu pracovat
        for (int i = 0; i < uart_data.leng; i++) {

                Serial.printf("x1: %d, y1: %d, x2: %d, y2: %d score: %d, color: %d, name: %d\n",
                            uart_data.results_array[i].x1,
                            uart_data.results_array[i].y1,
                            uart_data.results_array[i].x2,
                            uart_data.results_array[i].y2,
                            uart_data.results_array[i].score10,
                            uart_data.results_array[i].color,
                        uart_data.results_array[i].name);
                x1 =uart_data.results_array[i].x1;     // 30 min
                x2 =uart_data.results_array[i].x2;     // 280 max

                if(uart_data.results_array[i].color&&(!(uart_data.results_array[i].name)))
                {
                    int object_center = x1+((x2 - x1) / 2); //! Výpočet středu objektu do samostatné proměnné pro lepší čitelnost
                    Serial.printf("center je: %d",object_center);
                    if (object_center < 185 && object_center > 110) { //! Upraveno pro použití object_center
                        Serial.println("##### MAM TO #####");
                    }
                    else if (object_center > center) { //! Upraveno pro použití object_center
                                Serial.println("##### RRRRRRRRRRRRRRRRRRRRRRRRR #####");
                        tocka(-1, 100);
                        delay(300);
                    }
                    else if (object_center < center && object_center > 30) { //! Upraveno pro použití object_center
                        Serial.println("##### LLLLLLLLLLLLLLLLLLL #####");
                        tocka(1, 100);
                        delay(300);
                    }
                }
                else{
                    Serial.println("------------- je to modre -------------");
                }
        }
    }
    
    
    
}
    

    
 // I don't need loop, because i'm using while(true) in setup (it doesn't require to create global variables)x