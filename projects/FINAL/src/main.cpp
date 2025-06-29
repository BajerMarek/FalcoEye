//if (rkButtonIsPressed(BTN_DOWN)) { // Je tlačítko dolů stisknuté?
#include <iostream>
#include <Arduino.h>
#include <thread>

#include "RBCX.h"

#include "uart_comand.h"
#include"i2c_tools.h"
#include"senzor_com.h"

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_VL53L0X.h"

//#include "robotka.h"
// #include "SmartServoBus.hpp"
// using namespace lx16a; // aby nebylo třeba to psát všude
// static SmartServoBus servoBus;

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

unsigned long start_time =0;
unsigned long end_time =60000;

int enc_to_cm = 400; //! mm - 4000 převod na metr
int roztec = 175; //! mm -  vzdalenost středů kol od sebe
int r_kola = 36; //! mm -  poloměr kola 

int side = 1; //! 0 = modra, 1 =cervena 

//! servo 1 - dveře servo 2 - kufr

int stopper =0;
// ((poloměr + rozteč)*4000) × π × stupně // v metrech
//#include "robotka.h"
UARTResult_t uart_data;
I2cout senzor_data;
void STOP()
{
    auto& man = rb::Manager::get(); // get manager instance as singleton
    //int distance_1 = senzor_data.m1.RangeDMaxMilliMeter;
    //int distance_2 = senzor_data.m2.RangeDMaxMilliMeter;
            Serial.print("Senzor 1 (0x30): ");
        Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");
        Serial.print("Senzor 2 (0x31): ");
        Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    //man.motor(rb::MotorId::M1).power(-6000);
    //man.motor(rb::MotorId::M4).power(6000);
    //delay(400);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    delay(6000);
    stopper=0;
    //auto& man = rb::Manager::get(); // vytvoří referenci na man class
    // if(senzor_data.m1.RangeDMaxMilliMeter > distance_1 || senzor_data.m2.RangeDMaxMilliMeter > distance_2)
    // {

    // }
}

void turn(int angle, int rychlost, int lesser = 1)
{
    if(stopper) STOP();
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    //m1 musí být -
    int M1_pos = 0, smer= 1;
    int M4_pos = 0, odhylaka = 0, integral = 0;// last_odchylka =0, rampa_vzdalenost = 640;
    //int P =110, I = 0.01, D =0.5; 
    float cil = 0;
    cil = ((((roztec+r_kola))*PI*angle/2)/40)/lesser;    // roztec a kolo jsou v mm

    while(abs(M1_pos)<abs(cil)&& abs(M4_pos)<abs(cil))   //! 4000 převod na metry
    {
        if(stopper) STOP();
        if (cil<0) smer = -1;
        //else smer = 1;
       // man.motor(rb::MotorId::M1).setCurrentPosition(0);
        //man.motor(rb::MotorId::M4).setCurrentPosition(0);
        odhylaka = abs(M1_pos) - abs(M4_pos);
        integral += odhylaka; 

        man.motor(rb::MotorId::M1).power(smer*rychlost);
        man.motor(rb::MotorId::M4).power(smer*rychlost);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        delay(50);
        std::cout<<"cil: "<<cil<<" M1pos: "<<-1*M1_pos<<" M4 pos: "<<M4_pos<<std::endl;
        std::cout<<"odchylak: "<<M1_pos-M4_pos<<std::endl;

        delay(50);
        //last_odchylka = odhylaka;
    }
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

    odhylaka = 0, integral = 0;
    
}
void jizda_vpred(int cil,int rychlost)
{
    if(stopper) STOP();
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);

   //m1 musí být -
    int M1_pos = 0, M4_pos = 0, odhylaka = 0, integral = 0, last_odchylka =0, rampa_vzdalenost = 640; //součet dvou ramp 
    int P =110, I = 0.01, D =0.5; 
    int target = rychlost;
    int a = 500;
    int super_cil = cil*40 - rampa_vzdalenost;

    //! zrychlení
    for(int i = 0; i < target-1; i+=a)
    {
            if(stopper) STOP();
        odhylaka = M1_pos-M4_pos;
        integral += odhylaka; 
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        man.motor(rb::MotorId::M1).power(i+odhylaka*110);
        man.motor(rb::MotorId::M4).power(-i);
        delay(10);
        //! musím zjistit jakou vzdálenost tímto ujedu nasledně ji z dvojnasobit a odečíst do požadované vzdálenosti
    }
    integral =0;    //###
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    std::cout<<"M1pos: "<<-1*M1_pos<<" < "<<(super_cil)<<std::endl;
    while(-1*M1_pos<super_cil)   //! 4000 převod na metry
    {
            if(stopper) STOP();
        odhylaka = M1_pos-M4_pos;   // otoceni 1 a 4
        integral += odhylaka; 

        man.motor(rb::MotorId::M1).power(target+odhylaka*P+integral*I+(odhylaka-last_odchylka)*D);
        man.motor(rb::MotorId::M4).power(target*-1);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();

        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });

        delay(50);
        //Serial.printf("odhylaka: %d\n",odhylaka);
        std::cout<<"M1: "<<M1_pos<<" M4: "<<M4_pos<<std::endl;
        // if(odhylaka>1000 || odhylaka<-1000)
        // {
        //         man.motor(rb::MotorId::M1).setCurrentPosition(0);
        //         man.motor(rb::MotorId::M4).setCurrentPosition(0);
        //         odhylaka =0;
        // }
        last_odchylka = odhylaka;
        
    }
    //! zpomalení
    
    for(int i = target; i>=0; i-=a)
    {
            if(stopper) STOP();
        //odhylaka = M4_pos-M1_pos;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        man.motor(rb::MotorId::M1).power(i+odhylaka*0.01);
        man.motor(rb::MotorId::M4).power(-i);
        delay(10);
        //if(odhylaka>1000) odhylaka = 0;
        //std::cout<<"I: "<<i<<std::endl;
    }
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    odhylaka = 0;
}

bool vidim_puk(int side, UARTResult_t uart_data)
{
    for(int i = 0; i< uart_data.leng;i++)
        if(side == uart_data.results_array[i].color)
        {
            Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXx");
            return true;
        }
    return false;
}

void detekce_nepritele(int &stop)
{
    //auto& man = rb::Manager::get(); // vytvoří referenci na man class
    while (1)
    {
        if((senzor_data.m1.RangeMilliMeter==8191||senzor_data.m1.RangeMilliMeter==4) &&(senzor_data.m2.RangeMilliMeter==8191||senzor_data.m2.RangeMilliMeter==4))
        {
            stop =0;
            delay(50);
            continue;
        }
        if(((senzor_data.m1.RangeMilliMeter <100&&senzor_data.m1.RangeMilliMeter>4)|| (senzor_data.m2.RangeMilliMeter <300&&senzor_data.m2.RangeMilliMeter>4)))
        {
            //man.motor(rb::MotorId::M1).setCurrentPosition(0);
            //man.motor(rb::MotorId::M4).setCurrentPosition(0);

            stop = 1;
            //Serial.println("---- MAME NEPRITELE ----");
            sleep(50);
            //start += 1000;
        }
        else
        {
            stop = 0;
            delay(50);
        } 

        //Serial.printf("---- SLEDUJI A MAM: %d ----",stop);
    }
    
}

// cíl v cm
// rychlost v rozmezí -32768 až 32768
double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}
void jedu_pro_puk()
{
    if(stopper) STOP();
     Serial.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    int x1 =1;
    int x2 =1;
    int center = 125;
    int object_center =1;
    int zacatek_time = 0;


    while(!(object_center < 195 && object_center > 140))
    {

        if(stopper) STOP();
        if(uart_data.header == 0)
        {
            zacatek_time +=10;
            if(zacatek_time>=3000) break;
        }
        
        //!vypiš si střed
        // if(uart_data.header ==0)
        // {
        //     man.motor(rb::MotorId::M1).power(7000);
        //     man.motor(rb::MotorId::M4).power(-7000);
        //     delay(1000);

        // }
        // Serial.printf("##### millis()-zacatek_time: %d #####",millis()-zacatek_time);
        // if(millis()-zacatek_time >=konec_time)
        // {
        //     break;

        // }
        
        for (int i = 0; i < uart_data.leng; i++) 
        {
            if(stopper) STOP();
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
            object_center = x1+((x2 - x1) / 2); //! Výpočet středu objektu do samostatné proměnné pro lepší čitelnost

                    if (object_center > center) { //! Upraveno pro použití object_center
                                Serial.println("##### RRRRRRRRRRRRRRRRRRRRRRRRR #####");
                        //turn(1,4500,2);
                         if(stopper) STOP();
                            man.motor(rb::MotorId::M1).power(6150);
                            man.motor(rb::MotorId::M4).power(6150);
                        delay(10);
                    }
                    if (object_center < center) { //! Upraveno pro použití object_center
                        Serial.println("##### LLLLLLLLLLLLLLLLLLL #####");
                        //turn(-1,4500,2);
                         if(stopper) STOP();
                        man.motor(rb::MotorId::M1).power(-6150);
                        man.motor(rb::MotorId::M4).power(-6150);
                        //delay(100);
                         delay(10);
                    }
                    // else
                    // {
                    //     Serial.println("---- NIC NEVIDIM ----");
                    //     //man.motor(rb::MotorId::M1).power(-2000);
                    //     //man.motor(rb::MotorId::M4).power(2000);
                    //     delay(100);
                    // }
            
        }
        delay(10);
        //Serial.println("##### MAM TO #####");
        //uart_data = {0};
    }
    if(zacatek_time<3000)
    {
        Serial.println("##### MAM TO #####");
        turn(-1,7000,2);
        jizda_vpred(50,20000);
        
    }
    uart_data = {0};
    //!jinak konec
}
//! m1 - de do boku
void hledani_toceni_180()
{
    if(stopper) STOP();
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    //m1 musí být -
    int rychlost = 7000, angle = 80;
    float smer= 1.0;
    int M4_pos = 0, M1_pos = 0, odhylaka = 0, integral = 0;// last_odchylka =0, rampa_vzdalenost = 640;
    //int P =110, I = 0.01, D =0.5; 
    float cil = (((roztec+r_kola))*PI*angle/2)/40;    // roztec a kolo jsou v mm
    
    for(int I =0;I<4;I++)
    {
    if(stopper) STOP();
        if (cil<0) smer = -1.2;
        else smer = 1;
        Serial.printf("abs(M1_pos) %d < %f cil && abs(M4_pos) %d < %f cil\n",abs(M1_pos),abs(cil),abs(M4_pos),abs(cil));
        while(abs(M1_pos)<abs(cil)&& abs(M4_pos)<abs(cil))    //! 4000 převod na metry
        {
            if(vidim_puk(side,uart_data)) 
            {
                man.motor(rb::MotorId::M1).power(0);
                man.motor(rb::MotorId::M4).power(0);
                delay(1000);
                jedu_pro_puk();
                break;
            }

           // man.motor(rb::MotorId::M1).setCurrentPosition(0);
            //man.motor(rb::MotorId::M4).setCurrentPosition(0);
            odhylaka = abs(M1_pos) - abs(M4_pos);
            integral += odhylaka; 
    
            man.motor(rb::MotorId::M1).power(smer*rychlost);
            man.motor(rb::MotorId::M4).power(smer*rychlost);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
            //! získá encodery z motoru
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = info.position();
            });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = -info.position();
            });
            delay(50);
            std::cout<<"cil: "<<cil<<" M1pos: "<<-1*M1_pos<<" M4 pos: "<<M4_pos<<std::endl;
            std::cout<<"odchylak: "<<M1_pos-M4_pos<<std::endl;
    
            delay(50);
            //last_odchylka = odhylaka;
        }
        man.motor(rb::MotorId::M1).setCurrentPosition(0);
        man.motor(rb::MotorId::M4).setCurrentPosition(0);
        man.motor(rb::MotorId::M1).power(0);
        man.motor(rb::MotorId::M4).power(0);
        delay(1000);
        M4_pos = 0, M1_pos = 0;
        Serial.printf("---- 90 done ---- cil: %f i: %d",cil,I);
        cil = -1*cil;
        if(I==1)  cil = -1*cil;
    }


    odhylaka = 0, integral = 0;
}

void auto_servo()
{
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.stupidServo(0).setPosition(-0.925f); 
    delay(3000);
    if(side)
    {
        while (1)
        {
            if (senzor_data.r>110)
            {
                man.stupidServo(0).setPosition(-1.5); 
                delay(3000);
                man.stupidServo(0).setPosition(-0.925f); 
                delay(500);
            }
            if(senzor_data.b > 110)
            {
                man.stupidServo(0).setPosition(2.0f); 
                delay(2500);
                man.stupidServo(0).setPosition(-0.925f); 
                delay(2500);
            }
             delay(50);
        }
        
    }
    else
    {
        while (1)
        {
            if (senzor_data.b>110)
            {
                man.stupidServo(0).setPosition(-1.5); 
                delay(3000);
                man.stupidServo(0).setPosition(-0.925f); 
                delay(500);
            }
    
            if(senzor_data.r > 110)
            {
                man.stupidServo(0).setPosition(2.0f); 
                delay(800);
                man.stupidServo(0).setPosition(-0.925f); 
                delay(800);
            }
            delay(50);
        }

    }

}
void hledani_vpred(int vzdalenost, int rychlost)
{

    if(stopper) STOP();
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    //man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    micros(); // update overflow
    int M1_pos = 0, M4_pos = 0, odhylaka = 0, integral = 0, last_odchylka =0; //součet dvou ramp 
    int P =55, I = 0.01, D =0.25; 
    int target = 10000;
    int a = 500;
   // int stop = 1;

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);

    //! zrychlení
    for(int i = 0; i < target-1; i+=a)
    {
    if(stopper) STOP();
        Serial.print("Senzor 1 (0x30): ");
        Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");
        Serial.print("Senzor 2 (0x31): ");
        Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.printf("motor 1: %d | motor 4: %d \n", M1_pos,M4_pos);

        if(vidim_puk(side,uart_data))
        {
            //stop =0;
            break;
        }

        odhylaka = M1_pos-M4_pos;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        man.motor(rb::MotorId::M1).power(i+odhylaka*110);
        man.motor(rb::MotorId::M4).power(-i);
        delay(10);
        Serial.printf("M1: %d | M4 4: %d \n", M1_pos,M4_pos);
        //! musím zjistit jakou vzdálenost tímto ujedu nasledně ji z dvojnasobit a odečíst do požadované vzdálenosti
    }
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    while(abs(M1_pos)<100*40)   //! sem příjde podmínka na najití puku
    {    
        if(stopper) STOP();
        if(vidim_puk(side,uart_data))
        {
            //stop =0;
            break;
        }
        //if((senzor_data.m2.RangeMilliMeter<400) ) turn(90,10000); //&& (senzor_data.m2.RangeMilliMeter<400)
        Serial.print("Senzor 1 (0x30): ");
        Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");
        Serial.print("Senzor 2 (0x31): ");
        Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.printf("motor 1: %d | motor 4: %d \n", M1_pos,M4_pos);

        odhylaka = M1_pos-M4_pos;   // otoceni 1 a 4
        integral += odhylaka; 

        man.motor(rb::MotorId::M1).power(target+odhylaka*P+integral*I+(odhylaka-last_odchylka)*D);
        man.motor(rb::MotorId::M4).power(target*-1);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();

        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });

        delay(50);
        //Serial.printf("odhylaka: %d\n",odhylaka);
        std::cout<<"M1: "<<M1_pos<<" M4: "<<M4_pos<<std::endl;
        // if(odhylaka>1000 || odhylaka<-1000)
        // {
        //         man.motor(rb::MotorId::M1).setCurrentPosition(0);
        //         man.motor(rb::MotorId::M4).setCurrentPosition(0);
        //         odhylaka =0;
        // }
        last_odchylka = odhylaka;
    }
    Serial.println("MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");
    odhylaka = 0, integral = 0;
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    jedu_pro_puk();
    
}

void get_data(UARTResult_t vstup)
{
    while(1)
    {
        if( uart_recive(vstup))
        {
            memcpy(&uart_data,&vstup,sizeof (vstup));
            //delay(10);
            //Serial.println("-----------Přečten-------------------");
           
           
        // Serial.printf("Header: %d, Length: %d, Sum: %d\n",
        //     uart_data.header, uart_data.leng, uart_data.suma);
       }
       else
       {
        //vstup.header = 0;
        vstup = {0};
       }

       delay(10);
    }
}

void get_senzor_data()
{
    while(1)
    {
    // pinMode(TCS_SDA_pin, PULLUP);

    // pinMode(TCS_SCL_pin, PULLUP);

    // Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro predni senzor 


    // //tcs.begin(0x29,&Wire1);
    // if (!tcs.begin(0x29,&Wire1)) {
    //     Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
    //     while (1);
    // } else {
    //     //Serial.println("TCS34725 detekován.");
    // }


    //  float r, g, b;
    //  tcs.getRGB(&r, &g, &b);
    //  senzor_data.r = r;
    //  senzor_data.g = g;
    //  senzor_data.b = b;
    //  delay(10);
    tcs.getRGB(&senzor_data.r,&senzor_data.g,&senzor_data.b);

    // dalkové senzory
     sensor1.rangingTest(&senzor_data.m1, false);
     sensor2.rangingTest(&senzor_data.m2, false);
     delay(10);
    }
}

void servo_dance()
{
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    //Serial.begin(115200);
    man.stupidServo(0).setPosition(-1.5); 
    delay(3000);
    man.stupidServo(0).setPosition(-0.925f); 
    delay(3000);
    man.stupidServo(0).setPosition(2.0f); //-80 -60 90
    delay(3000);

    //! servo kufr
    man.stupidServo(1).setPosition(-0.1f); // -5 -90 -5
    delay(3000);
    man.stupidServo(1).setPosition(-2.0f); 
    delay(3000);
    
    man.stupidServo(1).setPosition(-0.1f);
    delay(3000);
    //rkServosSetPosition(1, ); // Servo 1 nastaví na 180°
    //delay(1000);
}

//točí se na základě času
void tocka(int smer, int cas)   // smer 1 -> pravo -1 -> levo
{    
    int target = 20000;
    int a = 2000;
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    for(int i = 0; i < target-1; i+=a)
    {

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {

        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {

        });
        man.motor(rb::MotorId::M1).power(i*smer);
        man.motor(rb::MotorId::M4).power(i*smer);
        delay(10);
        
    }
    //m1 musí být -
    //int M1_pos, M4_pos, odhylaka, integral, last_odchylka =0;
    //int target = 10000;
    //int a = 500;
    
    //for(int i =0; i<cas;i+=50)
    //{
        man.motor(rb::MotorId::M1).power(target*smer);
        man.motor(rb::MotorId::M4).power(target*smer);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
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
    for(int i = 0; i > target; i-=a)
    {
        man.motor(rb::MotorId::M1).power(i*smer);
        man.motor(rb::MotorId::M4).power(i*smer);
        delay(10);
        
    }
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

}
//! najede na stěnu
void stena()
{
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    //man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    micros(); // update overflow
    int M1_pos = 0, M4_pos = 0, odhylaka = 0, integral = 0, last_odchylka =0; //součet dvou ramp 
    int P =55, I = 0.01, D =0.25; 
    int target = 10000;
    int a = 500;

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);

    //! zrychlení
    for(int i = 0; i < target-1; i+=a)
    {
        Serial.print("Senzor 1 (0x30): ");
        Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");
        Serial.print("Senzor 2 (0x31): ");
        Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.printf("motor 1: %d | motor 4: %d \n", M1_pos,M4_pos);
        if((senzor_data.m1.RangeMilliMeter < 210) || (senzor_data.m2.RangeMilliMeter<300))
        {
            //stop =0;
            break;
        }
        odhylaka = M1_pos-M4_pos;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        man.motor(rb::MotorId::M1).power(i+odhylaka*110);
        man.motor(rb::MotorId::M4).power(-i);
        delay(10);
        Serial.printf("M1: %d | M4 4: %d \n", M1_pos,M4_pos);
        //! musím zjistit jakou vzdálenost tímto ujedu nasledně ji z dvojnasobit a odečíst do požadované vzdálenosti
    }
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    while(!(senzor_data.m1.RangeMilliMeter < 210) && !(senzor_data.m2.RangeMilliMeter<300))
    {
        Serial.print("Senzor 1 (0x30): ");
        Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");
        Serial.print("Senzor 2 (0x31): ");
        Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.printf("motor 1: %d | motor 4: %d \n", M1_pos,M4_pos);

        odhylaka = M1_pos-M4_pos;   // otoceni 1 a 4
        integral += odhylaka; 

        man.motor(rb::MotorId::M1).power(target+odhylaka*P+integral*I+(odhylaka-last_odchylka)*D);
        man.motor(rb::MotorId::M4).power(target*-1);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();

        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });

        delay(50);
        //Serial.printf("odhylaka: %d\n",odhylaka);
        std::cout<<"M1: "<<M1_pos<<" M4: "<<M4_pos<<std::endl;
        // if(odhylaka>1000 || odhylaka<-1000)
        // {
        //         man.motor(rb::MotorId::M1).setCurrentPosition(0);
        //         man.motor(rb::MotorId::M4).setCurrentPosition(0);
        //         odhylaka =0;
        // }
  
        last_odchylka = odhylaka;

    }
    odhylaka = 0, integral = 0;
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    //man.motor(rb::MotorId::M1).power(0);
    //man.motor(rb::MotorId::M4).power(0);
    //! doje ke stěně a jedne ze senzoru míří na stěnu

    while(senzor_data.m2.RangeMilliMeter<460)
    {
        man.motor(rb::MotorId::M1).power(target);
        man.motor(rb::MotorId::M4).power(target);
        delay(50);
    }

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
          man.motor(rb::MotorId::M1).power(27000);   //! levý motor
        man.motor(rb::MotorId::M4).power(-32000);
        delay(1500);
    //jizda_vpred(50,22000);
}
    
void corner ()
{
    turn(70,320000);     //! zjistit že se umí otočit u stěny jstly ne předělat funkci
}

void cesta_zpet()
{
    auto& man = rb::Manager::get(); // get manager instance as singleton
    stena();
        Serial.printf("red: %f, green: %f, blue: %f",senzor_data.r,senzor_data.g,senzor_data.b);
    Serial.print("######################\n");
    
    Serial.print("Senzor 1 (0x30): ");
    Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    Serial.print("Senzor 2 (0x31): ");
    Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    while(senzor_data.m2.RangeMilliMeter>400)
    {
            Serial.printf("red: %f, green: %f, blue: %f",senzor_data.r,senzor_data.g,senzor_data.b);
    Serial.print("######################\n");
    
    Serial.print("Senzor 1 (0x30): ");
    Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    Serial.print("Senzor 2 (0x31): ");
    Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
        // if((senzor_data.m2.RangeMilliMeter<300) &&(senzor_data.m1.RangeMilliMeter<150))
        // {
        //     corner();
        // }

        // while(1)
        // {
        //     Serial.println("---- KONEC ----");
        //     delay(300);
        // }
    
        //if(stopper) STOP();
        man.motor(rb::MotorId::M1).power(27000);   //! levý motor
        man.motor(rb::MotorId::M4).power(-32000);
        delay(50);
        // if(senzor_data.m2.RangeMilliMeter<400)
        // {
        //     man.motor(rb::MotorId::M1).power(0);   //! levý motor
        //     man.motor(rb::MotorId::M4).power(0);
        //     delay(3000);
        // }
    }

        man.motor(rb::MotorId::M1).brake(27000);   //! levý motor
        man.motor(rb::MotorId::M4).brake(32000);
        delay(1000);

        //if((side && senzor_data.r>150)||(!(side) && ((senzor_data.g== 98.076927)||(senzor_data.g== 91.071434))))
        //{
            //! otevření kufru
            man.stupidServo(1).setPosition(-2.0f); 
            delay(2000);
            //! popojezd v místě konce
            //turn(50,10000);
            man.motor(rb::MotorId::M1).power(20000);
            man.motor(rb::MotorId::M4).power(20000);
            delay(1000);
            man.motor(rb::MotorId::M1).power(0);
            man.motor(rb::MotorId::M4).power(0);
 

            //! odjezd z pole 
            //turn(30,10000);
            for(int i =0;i<100;i++)
            {
                man.motor(rb::MotorId::M1).power(1000);
                man.motor(rb::MotorId::M4).power(-1000);
                delay(20);

            }
            man.motor(rb::MotorId::M1).power(0);
            man.motor(rb::MotorId::M4).power(0);
            man.stupidServo(1).setPosition(-2.0f); 
            delay(3000);

            //!zavření kufru´ř´ř´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´´
            man.stupidServo(1).setPosition(-0.1f); // -5 -90 -5
            delay(300000);
        //}
}
void set_up_peripherals()
{
        
    //start_time=millis();
    Serial.println("---- Začíná set up ----");
    Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    //! inicalizace senzoru
    Serial.println("Wire (I2C) inicializováno na SDA=21, SCL=22");
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    delay(20);

    digitalWrite(XSHUT2, HIGH);
    delay(10);
    
    //! senzor 2 inicializace
    Serial.println("zacina inicializace senzoru 2");
    //delay(100);
    if (!sensor2.begin()) {
      Serial.println("Nepodařilo se spustit senzor 2");
      while (1);
    }
    delay(100);
    scan_i2c();

  //writeRegister(0x30,0x8A,0x31);
    sensor2.setAddress(0x31);
    scan_i2c();

    //! senzor 1 inicializace
    Serial.println("zacina inicializace senzoru 1");
    digitalWrite(XSHUT1, HIGH);
        if (!sensor1.begin()) {
      Serial.println("Nepodařilo se spustit senzor 1");
      while (1);
    }
    Serial.println("Podařilo se spustit senzor 1");
    scan_i2c();

    sensor1.setAddress(0x30);
    scan_i2c();   

    //!  barevný senzor inicalizace
    pinMode(TCS_SDA_pin, PULLUP);
    pinMode(TCS_SCL_pin, PULLUP);
    Serial.println("zacina inicializace barevného senzoru");
    Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro barevny senzor 

    tcs.begin(0x29,&Wire1);
    if (!tcs.begin(0x29,&Wire1)) {
        Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
        while (1);
    } else {
        Serial.println("TCS34725 detekován.");
    }



    Serial.println("---- VSE OK ----");
    //scan_i2c();
}
void hledani_90(int speed, float smer = 1.0)
{
    Serial.printf("--------------------------------------------------------------------");
        if(stopper) STOP();
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    //m1 musí být -
    int angle = 90;    //!90° do prava
    int M4_pos = 0, M1_pos = 0;// last_odchylka =0, rampa_vzdalenost = 640;
    //int P =110, I = 0.01, D =0.5; 
    float cil = (((roztec+r_kola))*PI*angle/2)/40;    // roztec a kolo jsou v mm
    
        //if(stopper) STOP();
        if (cil<0) smer = -1.2;
        else smer = 1.25;
        Serial.printf("abs(M1_pos) %d < %f cil && abs(M4_pos) %d < %f cil\n",abs(M1_pos),abs(cil),abs(M4_pos),abs(cil));
        while(abs(M1_pos)<abs(cil))//&& abs(M4_pos)<abs(cil)    //! 4000 převod na metry
        {
            if(vidim_puk(side,uart_data)) 
            {
                break;
            }

           // man.motor(rb::MotorId::M1).setCurrentPosition(0);
            //man.motor(rb::MotorId::M4).setCurrentPosition(0);
    
            man.motor(rb::MotorId::M1).power(smer*speed);
            man.motor(rb::MotorId::M4).power(smer*speed);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
            //! získá encodery z motoru
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = info.position();
            });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = -info.position();
            });
            delay(50);
            std::cout<<"cil: "<<cil<<" M1pos: "<<-1*M1_pos<<" M4 pos: "<<M4_pos<<std::endl;
            std::cout<<"odchylak: "<<M1_pos-M4_pos<<std::endl;
    
            delay(50);
            //last_odchylka = odhylaka;
        }
        jedu_pro_puk();
        man.motor(rb::MotorId::M1).setCurrentPosition(0);
        man.motor(rb::MotorId::M4).setCurrentPosition(0);
        man.motor(rb::MotorId::M1).power(0);
        man.motor(rb::MotorId::M4).power(0);
        delay(1000);
        M4_pos = 0, M1_pos = 0;
}
        

void homologace()
{
    auto& man = rb::Manager::get(); // get manager instance as singleton
            //     man.motor(rb::MotorId::M1).power(20000);
            // man.motor(rb::MotorId::M4).power(20000);
            // delay(2000);
            // delay(10000);
    hledani_vpred(100,20000);
    delay(3000);
    // man.motor(rb::MotorId::M1).setCurrentPosition(0);
    // man.motor(rb::MotorId::M4).setCurrentPosition(0);
    // man.motor(rb::MotorId::M1).power(0);
    // man.motor(rb::MotorId::M4).power(0);
    // delay(1000);
    hledani_90(6000,1.0);
    cesta_zpet();

    //delay(6000);
    //!turn(130,10000);
    if(stopper) STOP();
    

}

void setup() {
    Serial.begin(115200);
//     printf("Init manager instance\n");
    auto& man = rb::Manager::get(); // get manager instance as singleton
//     Serial.println("instance done");
//     man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
//     Serial.println("stáhnuto done");
//     micros(); // update overflow
//     Serial.println("micros done");
//     delay(200);

    
//     while(1)
//     {
//         Serial.println("---- CEKANI NA START ----");
//         if(man.buttons().right())
//         {

//             Serial.println("---- START SET UP ----");
            
//             set_up_peripherals();
//             delay(200);
            
//             uart_set_up();
//             delay(200);
            
//             Serial.println("servo test start");  
            
//             //servo_dance();
//             Serial.println("servo test done");
            
//             // Serial.println("Motor test start");
//             // man.motor(rb::MotorId::M1).power(-10000);
//             // man.motor(rb::MotorId::M4).power(10000);
//             // delay(2000);
//             // man.motor(rb::MotorId::M1).power(10000);
//             // man.motor(rb::MotorId::M4).power(-10000);
//             // delay(2000);
//             // man.motor(rb::MotorId::M1).power(0);
//             // man.motor(rb::MotorId::M4).power(0);
//             // Serial.println("Motor test done");
            
//             //! zapínání vláken
        
//             std::thread uart_thread (get_data, uart_data);
//             uart_thread.detach();
//             Serial.println("uart vlakno done");
        
//             std::thread i2c_thread(get_senzor_data);
//             i2c_thread.detach();
//             Serial.println("i2c vlakno done");

//             std::thread servo_thread(auto_servo);
//             servo_thread.detach();

//             //while(!(senzor_data.m1.RangeDMaxMilliMeter >0 && senzor_data.m2.RangeDMaxMilliMeter> 0)) delay(50);
//             Serial.println("---- START ----");
//             break;
//         }
//         delay(200);
//     }


//     // while(1)
//     // {
//     //     Serial.printf("%d",man.buttons().right());
//     //     if(man.buttons().right())
//     //     {
//     //         Serial.println("*******************************");
//     //         delay(1000);
//     //         //turn(180,10000);
//     //         //man.motor(rb::MotorId::M1).power(16000);
//     //         //jizda_vpred(50,20000);
//     //         //stena();
//     //         servo_dance();
//     //     }
//     //     delay(200);
//     // }

// }

    
        

        //turn(1,1);
        //start_time=millis();
        Serial.println("START");
        //rkConfig cfg;
        //rkSetup(cfg);
        Serial.println("RK hotovo");
        
        Serial.begin(115200);
        while (! Serial) {
        delay(1);  //! dat vedet, co se stalo 
        }
        
        //Wire.begin(21, 22);  // SDA = GPIO21, SCL = GPIO22
        Wire.begin(21, 22);
        delay(10);
        Serial.println("Wire (I2C) inicializováno na SDA=21, SCL=22");
        
        pinMode(XSHUT1, OUTPUT);
        pinMode(XSHUT2, OUTPUT);
        
        digitalWrite(XSHUT1, LOW);
        digitalWrite(XSHUT2, LOW);
        delay(20);
        
        digitalWrite(XSHUT2, HIGH);
        delay(10);
        //scan_i2c();
        
        
        Serial.println("zacina inicializace senzoru 2 #############");
        delay(100);
        if (!sensor2.begin()) {
          Serial.println("Nepodařilo se spustit senzor 2");
          while (1);
        }
        delay(100);
        scan_i2c();
        
        //writeRegister(0x30,0x8A,0x31);
        sensor2.setAddress(0x31);
        scan_i2c();
        
        
        Serial.println("zacina inicializace senzoru 1 #############");
        digitalWrite(XSHUT1, HIGH);
            if (!sensor1.begin()) {
          Serial.println("Nepodařilo se spustit senzor 1");
          while (1);
        }
        Serial.println("Podařilo se spustit senzor 1");
        scan_i2c();
        sensor1.setAddress(0x30);
        // writeRegister(0x29,0x8A,0x30);
        scan_i2c();   
        
        pinMode(TCS_SDA_pin, PULLUP);
        pinMode(TCS_SCL_pin, PULLUP);
        Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro barevny senzor 
        
        tcs.begin(0x29,&Wire1);
        if (!tcs.begin(0x29,&Wire1)) {
            Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
            while (1);
        } else {
            Serial.println("TCS34725 detekován.");
        }
        scan_i2c();
        
        Serial.println("RB3204-RBCX\n");
        printf("RB3204-RBCX\n");
        delay(50);
        printf("Init manager\n");
        
        
        //auto& man = rb::Manager::get(); // get manager instance as singleton
        Serial.println("instance done");
        man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
        Serial.println("stáhnuto done");
        
        micros(); // update overflow
        Serial.println("micros done");
                man.stupidServo(1).setPosition(-0.1f); // -5 -90 -5
        delay(1000);
        delay(200);
        
        uart_set_up();
        
        
        //delay(1000);
        
        
        //lox.begin();
        delay(200);
        
        //writeRegister(VL53L0X_DEFAULT_ADDR, I2C_SLAVE_DEVICE_ADDRESS, VL53L0X_NEW_ADDR);
        //delay(200);
        
        // Inicializace senzoru na nové adrese
        
        // Serial.println("Senzor VL53L0X uspesne inicializovan.");
        // tcs.begin(0x29);
        
        //uart_thread_function();
        
        std::thread uart_thread (get_data, uart_data);
        uart_thread.detach();
        
        std::thread i2c_thread(get_senzor_data);
        i2c_thread.detach();
        
        std::thread servo_thread(auto_servo);
        servo_thread.detach();
        
        std::thread detekce_thread(detekce_nepritele,std::ref(stopper));
        detekce_thread.detach();
        delay(1000);
        Serial.println("start");




    //servo_dance();

    // while(1)
    // {
    //     Serial.printf("%d",man.buttons().right());
    //     if(man.buttons().right())
    //     {
    //         Serial.println("*******************************");
    //         delay(1000);
    //         //turn(180,10000);
    //         //man.motor(rb::MotorId::M1).power(16000);
    //         //jizda_vpred(50,20000);
    //         //stena();
    //         servo_dance();
    //     }
    //     delay(200);
    // }

}
void loop()
{
    // auto& man = rb::Manager::get(); // get manager instance as singleton
    // while(1)
    // {
    //     Serial.printf("%d",man.buttons().right());
    //     if(man.buttons().right())
    //     {
    //         Serial.println("*******************************");
    //         delay(1000);
    //         //turn(180,10000);
    //         //man.motor(rb::MotorId::M1).power(16000);
    //         //jizda_vpred(50,20000);
    //         //stena();
    //         servo_dance();
    //     }
    //     delay(200);
    // }
    // while(1)
    // {
   // if (man.buttons().down()) {
    //    break;
   // }
    // delay(100);
   // }
//   cesta_zpet();
//    delay(10000);
//    hledani_vpred(50,20000);
//    hledani_toceni_180();
//     if(millis()-start_time >=end_time)
//     {
//         cesta_zpet();

//     }
    //cervena();
    Serial.printf("red: %f, green: %f, blue: %f",senzor_data.r,senzor_data.g,senzor_data.b);
    Serial.print("######################\n");
    if(senzor_data.r >100) side =1;
    else side =0;
    if(senzor_data.m1.RangeMilliMeter>0 && senzor_data.m2.RangeMilliMeter>0) homologace();
    delay(1000000);

}
    