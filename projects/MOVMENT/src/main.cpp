//if (rkButtonIsPressed(BTN_DOWN)) { // Je tlačítko dolů stisknuté?
#include <iostream>
#include <Arduino.h>
#include "RBCX.h"
#include "uart_comand.h"
//#include "robotka.h"
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
    int M1_pos, M4_pos, odhylaka, integral, last_odchylka =0;
    int target = 10000;
    int a = 500;
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    
    for(int i =0; i<cas;i+=50)
    {
        odhylaka = M1_pos-M4_pos;
        integral += odhylaka; 

        man.motor(rb::MotorId::M1).power(20000*smer);
        man.motor(rb::MotorId::M4).power((20000+odhylaka*55+integral*0.01+(odhylaka-last_odchylka)*0.1)*smer);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
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
    printf("RB3204-RBCX\n");
    delay(50);

    printf("Init manager\n");


    auto& man = rb::Manager::get(); // get manager instance as singleton
    man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    micros(); // update overflow
    delay(200);
    uart_set_up();
    // while (true) { // drive motor M1 to position 1000 with 100% power (32767) if button down is pressed and return to 0 position if button is released
    //     if (man.buttons().down()) {
    //         man.leds().green(true);
    //         man.setMotors().driveToValue(rb::MotorId::M1, 1000, 32767).set();
    //     } else {
    //         man.leds().green(false);
    //         man.setMotors().driveToValue(rb::MotorId::M1, 0, 32767).set();
    //     }
    // }
    // UARTResult_t vstup_uart;
    // int z=0,x1=0,x2=0,y1=0,y2 =0;
    // int LX, RX =0;
    // while (true) { // drive motor M1 to position 1000 with 100% power (32767) if button down is pressed and return to 0 position if button is released
    //  //   int i = man.motor(rb::MotorId::M1).position();
    //     //! puvodni program
    //     // if (man.buttons().down()) {
    //     //     printf("Tlacitko stisknuto\n");


    //     //     encoder(3000);

    // //     // //     printf("Tlacitko stisknuto\n");
    // //     // //     //delay(1000);
    // //     // //     man.motor(rb::MotorId::M1).power(-16000);
    // //     // //     man.motor(rb::MotorId::M4).power(16000);
    // //     // //     delay(3000);
    // //     // //     // man.setMotors().driveToValue(rb::MotorId::M1, 100*ticksToMm, -32767).set();
    // //     // //     // man.setMotors().driveToValue(rb::MotorId::M4, 100*ticksToMm, 32767).set();
    // //     // } else {
    // //     //     man.motor(rb::MotorId::M1).brake(16000);
    // //     //     man.motor(rb::MotorId::M4).brake(16000);
    // //     //     delay(3000);
    // //     // }
    // //     //!tady konci

    //     while(z<1)
    //     {
    //         if (get_uart_data(vstup_uart)) 
    //         {
    //             // Mám teď celý packet, můžu pracovat
    //             for (int i = 0; i < MAX_OD_BOX_CNT; i++) {
    //                 if(vstup_uart.results_array[i].score10>900)
    //                 {
    //                     Serial.printf("x1: %d, x2: %d, score: %d\n",
    //                                 vstup_uart.results_array[i].x1,
    //                                 vstup_uart.results_array[i].x2,
    //                                 vstup_uart.results_array[i].score10);
    //                     x1 += vstup_uart.results_array[i].x1;
    //                     //y1 += vstup_uart.results_array[i].y1;
    //                     x2 += vstup_uart.results_array[i].x2;
    //                     //y2 += vstup_uart.results_array[i].y2;
    //                     z =1;
    //                 }
    //             }
    //         }
    //     }
    //     delay(100);

    //     LX = x1 / 20;
    //     RX = x2 / 20;
    //     Serial.println("##########################################");
    //     Serial.printf("LX je : %d",LX);
    //     Serial.println("##########################################");
    //     if (RX<150&&RX>10)
    //     {
    //         tocka(1,100);
    //     }
    //     z=0,y1=0,y2=0,x1=0,x2=0;
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

void loop() {
    Serial.println("ping");
    delay(1000);
} // I don't need loop, because i'm using while(true) in setup (it doesn't require to create global variables)x