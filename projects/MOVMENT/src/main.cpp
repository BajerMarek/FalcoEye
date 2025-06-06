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
#include "SmartServoBus.hpp"
using namespace lx16a; // aby nebylo třeba to psát všude
static SmartServoBus servoBus;

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
unsigned long end_time =10000;



int enc_to_cm = 400; //! mm - 4000 převod na metr
int roztec = 175; //! mm -  vzdalenost středů kol od sebe
int r_kola = 36; //! mm -  poloměr kola 

int side = 0; //! 0 = modra, 1 =cervena 

int stopper =0;
// ((poloměr + rozteč)*4000) × π × stupně // v metrech
//#include "robotka.h"
UARTResult_t uart_data;
I2cout senzor_data;

// cíl v cm
// rychlost v rozmezí -32768 až 32768
double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

//! m1 je

double uhel_od_zdi(int m1, int m2)// vše v mm
{
    //double vysledek =1.0;
    int offset =120;
     const double angle_rad = PI / 4.0; // 45° v radiánech
    //vytvoří hodnoty pro senzor v xy souřadnicích 
    double sensorBx = offset * cos(angle_rad);
    double sensorBy = offset * sin(angle_rad);

    // Bod dopadu v xy souřadnicích
    double hitBx = sensorBx + m2 * cos(angle_rad);
    double hitBy = sensorBy + m2 * sin(angle_rad);

    //return vysledek;    // Výpočet bodu dopadu paprsku A (přímý směr – X osa)
    double hitAx = m1;
    double hitAy = 0.0;

    // Vektor od A k B (vektor rovnoběžný se stěnou)
    double vx = hitBx - hitAx;
    double vy = hitBy - hitAy;

    // Úhel mezi vektorem stěny a směrem senzoru A (vektor (1, 0))
    double angleToWall = atan2(vy, vx);  // v radiánech

    // Vrací absolutní hodnotu úhlu ve stupních
    //Serial.printf("hitA = (%.3f, %.3f), hitB = (%.3f, %.3f)\n",
      //        hitAx, hitAy, hitBx, hitBy);
    return fabs(radToDeg(angleToWall));
}

void vyrovnani_se_zdi (double uhel_k_stene)
{
    double suma =0;
    for(int i = 0; i< 10; i++)
    {
       suma += uhel_od_zdi(senzor_data.m1.RangeMilliMeter,senzor_data.m1.RangeMilliMeter);
       delay(15);
    }
    double avg = suma/10;
    if (avg<90&&avg>88) Serial.println("################ jsem rovno ################");
    Serial.printf("------ avg: %.3f",avg);
    delay(10);
}
void jizda_vpred(int cil,int rychlost)
{
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
void turn(int angle, int rychlost, int lesser = 1)
{
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
        vstup.header = 0;
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
    //Serial.begin(115200);
    Serial.println("servo start hotovo");

    servoBus.begin(2, UART_NUM_1, GPIO_NUM_27);
    Serial.println("servo begin hotovo");

    //servoBus.setId(1);
    delay(100);
    servoBus.set(1,30_deg,100);
    Serial.println("servo move hotovo");
    servoBus.set(0,170_deg,100);
    delay(2500);
    //     Serial.println("done");
    servoBus.set(1,240_deg,100);
    servoBus.set(0,90_deg,100);       // 100 otevřeno
    delay(2500);
    //  delay(100);
    //     Serial.println("done");
    servoBus.set(1,50_deg,100);
    servoBus.set(0,170_deg,100);       // 100 otevřeno
    delay(2500);

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
void kupredu(int M1_dis, int M4_dis)
{
    //m1 musí být -
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M4).power(16000);

}
void STOP(rb::Motor &m)
{
    Serial.println("---- KONEC VSEMU ----");
    //auto& man = rb::Manager::get(); // vytvoří referenci na man class
    m.brake(20000);
}

void zkouska(int angle,int speed)
{
    Serial.begin(115200);
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    micros(); // update overflow
    delay(200);
    int M1_pos =0;
    int M4_pos =0;
    while(1)
    {
        
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
        M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        Serial.printf("motor 1: %d | motor 4: %d \n", M1_pos,M4_pos);
        micros();
        delay(50);
    }
    //man.motor(rb::MotorId::M1).driveToValue()
}

void stena()
{
    stopper = 1;
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

    while(senzor_data.m2.RangeMilliMeter<400)
    {
        man.motor(rb::MotorId::M1).power(target);
        man.motor(rb::MotorId::M4).power(target);
        delay(50);
    }

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    stopper = 0;
    jizda_vpred(50,20000);
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
                delay(3000);
            }
            if(senzor_data.b > 110)
            {
                man.stupidServo(0).setPosition(2.0f); 
                delay(3000);
                man.stupidServo(0).setPosition(-0.925f); 
                delay(3000);
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
                delay(3000);
            }
    
            if(senzor_data.r > 110)
            {
                man.stupidServo(0).setPosition(2.0f); 
                delay(3000);
                man.stupidServo(0).setPosition(-0.925f); 
                delay(3000);
            }
            delay(50);
        }

    }

}
void detekce_nepritele(int stopper)
{
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    while (1)
    {
        if(((senzor_data.m1.RangeMilliMeter <150)|| (senzor_data.m2.RangeMilliMeter <150))&&!(stopper))
        {
            //man.motor(rb::MotorId::M1).setCurrentPosition(0);
            //man.motor(rb::MotorId::M4).setCurrentPosition(0);

            man.motor(rb::MotorId::M1).brake(20000);
            man.motor(rb::MotorId::M4).brake(20000);
            Serial.println("---- MAME NEPRITELE ----");
            sleep(50);
            //start += 1000;
        }
        delay(50);
    }
    
}
void setup() {
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

    
    auto& man = rb::Manager::get(); // get manager instance as singleton
    Serial.println("instance done");
    man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    Serial.println("stáhnuto done");

    micros(); // update overflow
    Serial.println("micros done");

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

    std::thread detekce_thread(detekce_nepritele,stopper);
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
        auto& man = rb::Manager::get(); // get manager instance as singleton
    //stena();
    //Serial.println("konec poksne funkce");
    //delay(10000);
    // while(1)
    // {
   // if (man.buttons().down()) {
    //    break;
   // }
    // delay(100);
   // }
    // if(millis()-start_time >=end_time)
    // {
    //     while(1)
    //     {
    //        Serial.print("---- HOTOVO ----\n"); 
    //     }
    // }
    //cervena();
    Serial.printf("red: %f, green: %f, blue: %f",senzor_data.r,senzor_data.g,senzor_data.b);
    Serial.print("######################\n");
    
    Serial.print("Senzor 1 (0x30): ");
    Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    Serial.print("Senzor 2 (0x31): ");
    Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    delay(150);
    //Serial.printf("uhel dozdi : %3f",uhel_od_zdi(senzor_data.m1.RangeMilliMeter,senzor_data.m2.RangeMilliMeter));
    //vyrovnani_se_zdi(uhel_od_zdi(senzor_data.m1.RangeMilliMeter,senzor_data.m2.RangeMilliMeter));
    
    //int diff_sen = abs(senzor_data.m1.RangeMilliMeter-senzor_data.m2.RangeMilliMeter);
    //float avg_sen = (senzor_data.m1.RangeMilliMeter+senzor_data.m2.RangeMilliMeter)/2;
    //     Serial.printf("---- diff %d ----",diff_sen);
    //     Serial.printf("---- avg %f ---- (avg/9.5)  %f ---- (avg/4.1) %f ----\n",avg_sen, (avg_sen/9.5),(avg_sen/4.1));
    //     if((diff_sen>(avg_sen/9.5))&&(diff_sen<(avg_sen/4.1))) Serial.printf("---- JSEM ROVNE ----\n");

    jizda_vpred(50,20000);
    Serial.println("---- UART START ----");

    int x1 =0;
    int x2 =0;
    int center = 125;
    if(man.buttons().right())
    {
        Serial.println("---- RESCUE START ----");
        delay(1000);
        jizda_vpred(50,20000);
    }
    if(uart_data.header == 255)
    {
        Serial.printf("Header: %d, Length: %d, Sum: %d\n",
            uart_data.header, uart_data.leng, uart_data.suma);
            
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

                if((!(uart_data.results_array[i].name))) // uart_data.results_array[i].color&&
                {
                    int object_center = x1+((x2 - x1) / 2); //! Výpočet středu objektu do samostatné proměnné pro lepší čitelnost
                    Serial.printf("center je: %d",object_center);
                    if (object_center < 195 && object_center > 140) { //! Upraveno pro použití object_center puvodně 180
                        Serial.println("##### MAM TO #####");
                        //jizda_vpred(50,20000);
                    }
                    else if (object_center > center) { //! Upraveno pro použití object_center
                                Serial.println("##### RRRRRRRRRRRRRRRRRRRRRRRRR #####");
                        // turn(1,7000,2);
                        // delay(300);
                    }
                    else if (object_center < center && object_center > 30) { //! Upraveno pro použití object_center
                        Serial.println("##### LLLLLLLLLLLLLLLLLLL #####");
                        // turn(-1,7000,2);
                        // delay(300);
                    }
                }
                else{
                    Serial.println("------------- nemam nic -------------");
                }
                uart_data = {0};
        }
    }
      
}
    