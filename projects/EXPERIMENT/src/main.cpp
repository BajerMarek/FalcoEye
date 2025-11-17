// if (rkButtonIsPressed(BTN_DOWN)) { // Je tlačítko dolů stisknuté?
#include <iostream>

#include <Arduino.h>
#include <thread>

#include "RBCX.h"
// #include "rbdns.h"  //pidn pro rychlejsi debug

#include "uart_comand.h"
#include "i2c_tools.h"
#include "senzor_com.h"

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_VL53L0X.h"

// deklarace instanci senzoru
#define I2C_SLAVE_DEVICE_ADDRESS 0x8A // Registr pro změnu adresy
#define VL53L0X_DEFAULT_ADDR 0x29     // Výchozí adresa senzoru
#define VL53L0X_NEW_ADDR 0x31         // Požadovaná nová adresa

#define XSHUT1 33
#define XSHUT2 13 // ten blize

Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
static const uint8_t TCS_SDA_pin = 14;
static const uint8_t TCS_SCL_pin = 26;

unsigned long start_time = 0;
unsigned long end_time = 60000;
//! 1000 encoderu je 270 mm - prevod je 3,7
float enc_to_cm = 3.7; //! mm -
int roztec = 175;      //! mm -  vzdalenost středů kol od sebe
int r_kola = 36;       //! mm -  poloměr kola

int side = 0; //! 0 = modra, 1 =cervena

//! servo 1 - dveře servo 2 - kufr

int stopper = 0;
// ((poloměr + rozteč)*4000) × π × stupně // v metrech
// #include "robotka.h"
UARTResult_t uart_data;
I2cout senzor_data;
// zastavý program na 6000ms
auto &man = rb::Manager::get(); // vytvoří referenci na man class
void STOP()
{
    // auto& man = rb::Manager::get(); // get manager instance as singleton

    // Serial.print("Senzor 1 (0x30): ");
    // Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    // Serial.print("\n");
    // Serial.print("Senzor 2 (0x31): ");
    // Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    // man.motor(rb::MotorId::M1).power(0);
    // man.motor(rb::MotorId::M4).power(0);
    // delay(6000);
    // stopper=0;
}
void toceni_dle_uhlu(float angle, int rychlost)
{
    int M1_pos = 0, M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0; //  rampa_vzdalenost = 640; 
    int M1_pred = 0, M4_pred = 0;
    float P = 4;
    float I = 0.001, D = 0.75;
    // int P =110, I = 0.01, D =0.5;
    int M4_smer = 1, M1_smer =1, smer =1;
    int ramp_distance = 0, a = 2000;
    double cil = 0;
    cil = ((PI * roztec) / (360 / abs(angle))) * 3.7; // roztec a kolo jsou v mm 

    if (angle < 0)  //M1 stoupá
    {
        smer = -1;
    }
    else            //M4 stoupá
    {
        smer = 1;
    }

    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    delay(100);
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });
    delay(100); 

    for (int i = 0; i < rychlost; i += 2000)
    {
        if (stopper)
            STOP();

        if(!(smer)&&(M1_pos>0))
        {
            M1_smer=-1;
        }
        
        else 
            M1_smer = 1;

        if(smer&&(M4_pos>0))
        {
            M4_smer=-1;
        }
        else
            M4_smer =1;
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });

        odchylka = abs(abs(M4_pred)-M4_smer*abs(M4_pos))  - abs(abs(M4_pred)-M4_smer*abs(M4_pos)); //!odchylka = (M1_pos-M1_pred) + (M4_pos-M4_pred);
        integral += odchylka;

        man.motor(rb::MotorId::M1).power(i + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        man.motor(rb::MotorId::M4).power(i + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        Serial.printf("##### i+odchylka*P = %d | a = %d | M1_pos = %d | M4_pos = %d #####\n", i + odchylka * P, a, M1_pos, M4_pos);
        last_odchylka = odchylka;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;

        delay(20);
    }
    ramp_distance = M1_pos - M1_pred;
    while ((abs(abs(M1_pred) - M1_smer*abs(M1_pos)) < cil-ramp_distance) || (abs(abs(M4_pred)-M4_smer*abs(M4_pos)) < cil-ramp_distance)) //! odstranění cil - ramp_distance
    {
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });
        delay(50);
        if(!(smer)&&(M1_pos>0))
        {
            M1_smer=-1;
        }
        
        else 
            M1_smer = 1;

        if(smer&&(M4_pos>0))
        {
            M4_smer=-1;
        }
        else
            M4_smer =1;
        odchylka = abs(abs(M1_pred) - M1_smer*abs(M1_pos)) - abs(abs(M4_pred)-M4_smer*abs(M4_pos)) ; //!odchylka = (M1_pos-M1_pred) + (M4_pos-M4_pred);

        //!odchylka = M1_pos + M4_pos; // otoceni 1 a 4
        integral += odchylka;
        // man.motor(rb::MotorId::M1).setCurrentPosition(0);
        // man.motor(rb::MotorId::M4).setCurrentPosition(0);

        if ((abs(abs(M1_pred) - M1_smer*abs(M1_pos)) <= cil-ramp_distance))//! odstranění cil - ramp_distance
        {
            man.motor(rb::MotorId::M1).power(smer * rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        }
        else man.motor(rb::MotorId::M1).power(0);
        if ((abs(abs(M4_pred) - M4_smer*abs(M4_pos)) <= cil-ramp_distance))//! odstranění cil - ramp_distance
        {
            man.motor(rb::MotorId::M4).power(smer * rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        }
        else man.motor(rb::MotorId::M4).power(0);
        //! získá encodery z motoru
        // std::cout<<"cil: "<<abs(cil+M1_pred)<<" M1pos: "<<-1*M1_pos<<" M4 pos: "<<M4_pos<<std::endl;
        //! std::cout<<"odchylka: "<<M1_pos-M4_pos<<std::endl;
        Serial.printf("M1_pred = %d | M4_pred = %d | M1_pos = %d | M4_pos = %d | M1_pos-M1_pred = %d | Odchylka = %d | M4_pos-M4_pred = %d | speed = %f | smer = %d \n", M1_pred, M4_pred, M1_pos, M4_pos, abs(M1_pos - M1_pred), odchylka, abs(M4_pos - M4_pred), rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D,smer);
        last_odchylka = odchylka;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;

        delay(10);
        // last_odchylka = odchylka;
    }
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

    odchylka = 0, integral = 0;
}

void jizda_vpred(float cil, int rychlost)
{
    if (stopper)
        STOP();
    auto &man = rb::Manager::get(); // vytvoří referenci na man class

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    // M4 má zápornou hodnotu
    int M1_pos = 0, M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0, rampa_vzdalenost = 640; // 800cm součet dvou ramp
    int M1_pred = 0, M4_pred = 0;
    float P = 50;
    float I = 0.0001, D = 1;
    // float D =0.5;
    int target = rychlost;
    int a = 2000;
    int ramp_distance = 0;
    cil = cil * 3.7;

    // man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
    //                                        { M4_pred = info.position(); });
    // man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
    //                                        { M1_pred = -info.position(); });

    //! zrychlení - rampa
    for (int i = 0; i < target; i += 2000)
    {
        if (stopper)
        {
            STOP();

        }
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
            { M4_pos = info.position(); });
            
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
            { M1_pos = -info.position(); });
            
            
            odchylka = M1_pos - M4_pos; // otoceni 1 a 4
            integral += odchylka;

        man.motor(rb::MotorId::M1).power(i);
        man.motor(rb::MotorId::M4).power(-i  - odchylka * P - integral * I - (odchylka - last_odchylka) * D);
        Serial.printf("##### i+odchylka*P = %d | a = %d | M1_pos = %d | M4_pos = %d #####\n", i + odchylka * P, a, M1_pos, M4_pos);

        delay(20);
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;
        last_odchylka = odchylka;
        //! musím zjistit jakou vzdálenost tímto ujedu nasledně ji z dvojnasobit a odečíst do požadované vzdálenosti
    }
    ramp_distance = M1_pos - M1_pred;
    integral = 0; // ###
    last_odchylka=0;
    //         man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
    //     M4_pred = -info.position();
    // });

    //     man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
    //     M1_pred = info.position();
    // });
    std::cout << "M1pos: " << M1_pos << " < " << cil + M1_pred << std::endl;

    Serial.printf("##### cil+M1_pred = %f #####\n", cil + M1_pred - ramp_distance);
    Serial.printf("#####  M1_pred = %d | M4_pred = %d #####\n", M1_pred, M4_pred);
    while (((M1_pos - M1_pred) < cil - ramp_distance) || ((M4_pos - M4_pred) < cil -ramp_distance)) //! Provedl jsem upravu od funkčního originalu odstarnil jsem + Mn_pred od cíle
    {
        if (stopper)
        {
            STOP();

        }

        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = info.position(); });

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = -info.position(); });
        delay(10);
        odchylka = M1_pos - M4_pos; // otoceni 1 a 4
        integral += odchylka;

        if ((M1_pos - M1_pred) < cil - ramp_distance)
            man.motor(rb::MotorId::M1).power(target);
        else{
            man.motor(rb::MotorId::M1).power(0);
        }
        if ((M4_pos - M4_pred) < cil - ramp_distance)
            man.motor(rb::MotorId::M4).power(-target - odchylka * P - integral * I - (odchylka - last_odchylka) * D); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
                                                                                                                      //! získá encodery z motoru
        else{
            man.motor(rb::MotorId::M4).power(0);
        }

        Serial.printf("M1_pos = %d | M4_pos = %d | speed = %f | M1_pos-M1_pred = %d | M1_pred = %d | M4_pos-M4_pred = %d | M4_pred = %d | Odchylka = %d\n", M1_pos, M4_pos, target + odchylka * P + integral * I + (odchylka - last_odchylka) * D, M1_pos - M1_pred, M1_pred, M4_pos - M4_pred, M4_pred, odchylka);
        delay(10);
        // std::cout<<"M1: "<<M1_pos<<" M4: "<<M4_pos<<std::endl;
        if (integral > 10000) integral = 10000;
        if (integral < -10000) integral = -10000;
        last_odchylka = odchylka;
    }

    //! zpomalení
    integral = 0; // ###
    last_odchylka=0;
    a = 2000;
    for (int i = target; i + odchylka * P + integral * I + (odchylka - last_odchylka) * D > 0; i -= a )
    {

   if (stopper)
            STOP();
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
            { M4_pos = info.position(); });
            
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
            { M1_pos = -info.position(); });
            
            
            odchylka = M1_pos - M4_pos; // otoceni 1 a 4
            integral += odchylka;

        man.motor(rb::MotorId::M1).power(i);
        man.motor(rb::MotorId::M4).power(-i  - odchylka * P - integral * I - (odchylka - last_odchylka) * D);
        Serial.printf("##### i+odchylka*P = %d | a = %d | M1_pos = %d | M4_pos = %d #####\n", i + odchylka * P, a, M1_pos, M4_pos);

        delay(20);
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;
        last_odchylka = odchylka;
    }
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    odchylka=0,integral=0,last_odchylka=0;
}
// vrátí true, když barva najitého puku je stejná jako strana
bool vidim_puk(int side, UARTResult_t uart_data)
{
    for (int i = 0; i < uart_data.leng; i++)
        if (side == uart_data.results_array[i].color)
        {
            Serial.println("-------------------------------- vidim puk --------------------------------");
            return true;
        }
    return false;
}
// Naměříly senzor m1 nebo m2 hodnotu která je menší jak 100 nebo maximální 8191 nebo 4 -> chyba měření
void detekce_nepritele(int &stop)
{
    while (1)
    {
        if ((senzor_data.m1.RangeMilliMeter == 8191 || senzor_data.m1.RangeMilliMeter == 4) && (senzor_data.m2.RangeMilliMeter == 8191 || senzor_data.m2.RangeMilliMeter == 4))
        {
            stop = 0;
            delay(50);
            continue;
        }
        if (((senzor_data.m1.RangeMilliMeter < 100 && senzor_data.m1.RangeMilliMeter > 4) || (senzor_data.m2.RangeMilliMeter < 300 && senzor_data.m2.RangeMilliMeter > 4)))
        {
            stop = 1;
            sleep(50);
        }
        else
        {
            stop = 0;
            delay(50);
        }
    }
}
// Převod na radiány
double radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}
// puk byl spatřen a pokd není ve středu robot se bude točit kolem do kola dokuďn nebude uprostřed
bool jedu_pro_puk(bool get_back = false, int distance_traveled = 0, float angle = 0, int direction = 1)
{
    if (stopper)
        STOP();
    Serial.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
    auto &man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

    int P = 25;
    float I = 0.001, D = 0.5;
    int M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0; //  rampa_vzdalenost = 640; 

    int x1 = 1;
    int x2 = 1;
    int center = 125;
    int object_center = 1;
    int zacatek_time = 0;
    int M1_pos = 0, M1_pred_tocenim = 0; // součet dvou ramp
    int last_direction = 1;              //! 1 =  levo -1 =pravo
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred_tocenim = info.position(); });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pos = info.position(); });
    // angle = 0;
    while (!(object_center < 195 && object_center > 140))
    {
        if (stopper)
            STOP();
        if (uart_data.header == 0)
        {
            zacatek_time += 10;
            printf("cas_cekani : %d\n", zacatek_time);
            if (zacatek_time >= 1500)
                break;
        }

        for (int i = 0; i < uart_data.leng; i++)
        {
            if (stopper)
                STOP();
            Serial.printf("x1: %d, y1: %d, x2: %d, y2: %d score: %d, color: %d, name: %d\n",
                          uart_data.results_array[i].x1,
                          uart_data.results_array[i].y1,
                          uart_data.results_array[i].x2,
                          uart_data.results_array[i].y2,
                          uart_data.results_array[i].score10,
                          uart_data.results_array[i].color,
                          uart_data.results_array[i].name);

            x1 = uart_data.results_array[i].x1;   // 30 min
            x2 = uart_data.results_array[i].x2;   // 280 max
            object_center = x1 + ((x2 - x1) / 2); //! Výpočet středu objektu do samostatné proměnné pro lepší čitelnost

            odchylka = abs(M1_pos) - abs(M4_pos); // otoceni 1 a 4
            integral += odchylka;
            if ((object_center > center))
            { //! Upraveno pro použití object_center
                Serial.println("##### RRRRRRRRRRRRRRRRRRRRRRRRR #####");
                // turn(1,4500,2);
                if (stopper)
                    STOP();
                man.motor(rb::MotorId::M1).power(7000 + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
                man.motor(rb::MotorId::M4).power(7000 + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
                // man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {M1_pos = info.position();});
                last_direction = -1;
                delay(10);
            }
            if (object_center < center)
            { //! Upraveno pro použití object_center
                Serial.println("##### LLLLLLLLLLLLLLLLLLL #####");
                // turn(-1,4500,2);
                if (stopper)
                    STOP();
                man.motor(rb::MotorId::M1).power(-7000 - odchylka * P - integral * I - (odchylka - last_odchylka) * D);
                man.motor(rb::MotorId::M4).power(-7000 - odchylka * P - integral * I - (odchylka - last_odchylka) * D);
                // delay(100);
                last_direction = 1;
                delay(10);
            }
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                                   { M1_pos = info.position(); });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                                   { M4_pos = -info.position(); });
        }

        delay(10);
    }
    if (M1_pos - M1_pred_tocenim != 0)
    {
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });

        Serial.println("##### MAM TO #####");
        // toceni_dle_uhlu(-1,7000);

        jizda_vpred(200, 15000);
        delay(1000);
        man.stupidServo(0).setPosition(-0.925f);
        delay(500);
        man.stupidServo(0).setPosition(0);
        delay(500);
        if (get_back == false)
            return true;
        if (get_back == true)
        {
            man.motor(rb::MotorId::M1).power(0);
            man.motor(rb::MotorId::M4).power(0);
            Serial.printf("##### angle = %f #####\n", angle);
            Serial.printf("##### M1_pos = %d #####\n", M1_pos);
            Serial.printf("##### M1_pred_tocenim = %d #####\n", M1_pred_tocenim);
            Serial.printf("##### M1_pos-M1_pred_tocenim = %d #####\n", M1_pos - M1_pred_tocenim);
            direction = -1;
            angle = direction * (angle + (abs(M1_pos - M1_pred_tocenim) * 360) / (3.7 * PI * roztec)); //!- 90;

            Serial.printf("##### angle = %f #####\n", angle);
            toceni_dle_uhlu(180, 15000);
            delay(1000);
            jizda_vpred(200, 15000);
            delay(1000);
            if(abs(angle)>10) toceni_dle_uhlu(angle, 15000);
            else delay(1000);

            jizda_vpred(distance_traveled, 15000);
            delay(1000);

            toceni_dle_uhlu(180, 15000);
            Serial.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
            return true;
        }
    }
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    uart_data = {0};
    return false;
    //! jinak konec
}
//! m1 - de do boku

void hledani_toceni_180()
{
    if (stopper)
        STOP();
    auto &man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    // m1 musí být -
    int rychlost = 7000, angle = 80;
    float smer = 1.0;
    int M4_pos = 0, M1_pos = 0, odchylka = 0, integral = 0; // last_odchylka =0, rampa_vzdalenost = 640;
    // int P =110, I = 0.01, D =0.5;
    float cil = ((PI * roztec) / (360 / angle)) * 3.7; // roztec a kolo jsou v mm

    for (int I = 0; I < 4; I++)
    {
        if (stopper)
            STOP();
        if (cil < 0)
            smer = -1.2;
        else
            smer = 1;
        Serial.printf("abs(M1_pos) %d < %f cil && abs(M4_pos) %d < %f cil\n", abs(M1_pos), abs(cil), abs(M4_pos), abs(cil));
        while ((abs(M1_pos) < abs(cil)) || (abs(M4_pos) < abs(cil)))
        {
            if (vidim_puk(side, uart_data))
            {
                man.motor(rb::MotorId::M1).power(0);
                man.motor(rb::MotorId::M4).power(0);
                delay(1000);
                jedu_pro_puk();
                break;
            }

            // man.motor(rb::MotorId::M1).setCurrentPosition(0);
            // man.motor(rb::MotorId::M4).setCurrentPosition(0);
            odchylka = abs(M1_pos) - abs(M4_pos);
            integral += odchylka;

            man.motor(rb::MotorId::M1).power(smer * rychlost);
            man.motor(rb::MotorId::M4).power(smer * rychlost); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
            //! získá encodery z motoru
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                                   { M1_pos = info.position(); });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                                   { M4_pos = -info.position(); });
            delay(50);
            std::cout << "cil: " << cil << " M1pos: " << -1 * M1_pos << " M4 pos: " << M4_pos << std::endl;
            std::cout << "odchylak: " << M1_pos - M4_pos << std::endl;

            delay(50);
            // last_odchylka = odchylka;
        }
        man.motor(rb::MotorId::M1).setCurrentPosition(0);
        man.motor(rb::MotorId::M4).setCurrentPosition(0);
        man.motor(rb::MotorId::M1).power(0);
        man.motor(rb::MotorId::M4).power(0);
        delay(1000);
        M4_pos = 0, M1_pos = 0;
        Serial.printf("---- 90 done ---- cil: %f i: %d", cil, I);
        cil = -1 * cil;
        if (I == 1)
            cil = -1 * cil;
    }

    odchylka = 0, integral = 0;
}
//! Servo pro odhazování puků
void auto_servo()
{
    auto &man = rb::Manager::get(); // vytvoří referenci na man class
    man.stupidServo(0).setPosition(-0.925f);
    delay(3000);
    if (side)
    {
        while (1)
        {
            if (senzor_data.r > 110)
            {
                man.stupidServo(0).setPosition(-1.5);
                delay(3000);
                man.stupidServo(0).setPosition(-0.925f);
                delay(500);
            }
            if (senzor_data.b > 110)
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
            if (senzor_data.b > 110)
            {
                man.stupidServo(0).setPosition(-1.5);
                delay(3000);
                man.stupidServo(0).setPosition(-0.925f);
                delay(500);
            }

            if (senzor_data.r > 110)
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
    if (stopper)
        STOP();
    auto &man = rb::Manager::get();                                            // vytvoří referenci na man class
    micros();                                                                  // update overflow
    int M1_pos = 0, M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0, rampa_vzdalenost = 640; // 800cm součet dvou ramp
    int M1_pred = 0, M4_pred = 0;
    float P = 75;
    float I = 0.0001, D = 1;
    // float D =0.5;
    int target = rychlost;
    int a = 2000;
    int ramp_distance = 0;
    int cil = vzdalenost * 3.7;
    bool puk_zpatren = false;
    // int stop = 1;
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = info.position(); });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = -info.position(); });

    //! zrychlení - rampa
 for (int i = 0; i < target; i += 2000)
    {
        if (stopper)
        {
            STOP();

        }

        if (vidim_puk(side, uart_data))
        {
            puk_zpatren=true;
            break;
        }
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
            { M4_pos = info.position(); });
            
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
            { M1_pos = -info.position(); });
            
            
            odchylka = M1_pos - M4_pos; // otoceni 1 a 4
            integral += odchylka;

        man.motor(rb::MotorId::M1).power(i);
        man.motor(rb::MotorId::M4).power(-i  - odchylka * P - integral * I - (odchylka - last_odchylka) * D);
        Serial.printf("##### i+odchylka*P = %d | a = %d | M1_pos = %d | M4_pos = %d #####\n", i + odchylka * P, a, M1_pos, M4_pos);

        delay(20);
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;
        last_odchylka = odchylka;
        //! musím zjistit jakou vzdálenost tímto ujedu nasledně ji z dvojnasobit a odečíst do požadované vzdálenosti
    }
    ramp_distance = M1_pos - M1_pred;
    integral = 0; // ###
    last_odchylka=0;    //         man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
    //     M4_pred = -info.position();
    // });

    //     man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
    //     M1_pred = info.position();
    // });

    std::cout << "M1pos: " << M1_pos << " < " << cil + M1_pred << std::endl;

    Serial.printf("##### cil+M1_pred = %f #####\n", cil + M1_pred - ramp_distance);
    Serial.printf("#####  M1_pred = %d | M4_pred = %d #####\n", M1_pred, M4_pred);
    while (((M1_pos - M1_pred) < cil - ramp_distance) || ((M4_pos - M4_pred) < cil -ramp_distance)) //! Provedl jsem upravu od funkčního originalu odstarnil jsem + Mn_pred od cíle
    {
        if (stopper)
        {
            STOP();

        }

        if(puk_zpatren) break;
        if (stopper)
            STOP();

        if (vidim_puk(side, uart_data))
        {
            puk_zpatren = true;
            break;
        }
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = info.position(); });

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = -info.position(); });
        delay(10);
        odchylka = M1_pos - M4_pos; // otoceni 1 a 4
        integral += odchylka;

        if ((M1_pos - M1_pred) < cil - ramp_distance)
            man.motor(rb::MotorId::M1).power(target);
        else{
            man.motor(rb::MotorId::M1).power(0);
        }
        if ((M4_pos - M4_pred) < cil - ramp_distance)
            man.motor(rb::MotorId::M4).power(-target - odchylka * P - integral * I - (odchylka - last_odchylka) * D); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
                                                                                                                      //! získá encodery z motoru
        else{
            man.motor(rb::MotorId::M4).power(0);
        }

        Serial.printf("M1_pos = %d | M4_pos = %d | speed = %f | M1_pos-M1_pred = %d | M1_pred = %d | M4_pos-M4_pred = %d | M4_pred = %d | Odchylka = %d\n", M1_pos, M4_pos, target + odchylka * P + integral * I + (odchylka - last_odchylka) * D, M1_pos - M1_pred, M1_pred, M4_pos - M4_pred, M4_pred, odchylka);
        delay(10);
        // std::cout<<"M1: "<<M1_pos<<" M4: "<<M4_pos<<std::endl;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;
        last_odchylka = odchylka;
    }

    //! zpomalení
    integral = 0; // ###
    last_odchylka=0;
    a = 2000;    //! zpomalení


  for (int i = target; i + odchylka * P + integral * I + (odchylka - last_odchylka) * D > 0; i -= a )
    {

        if (vidim_puk(side, uart_data))
        {
            puk_zpatren = true;
            break;
        }
        if (stopper)
            STOP();
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
            { M4_pos = info.position(); });
            
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
            { M1_pos = -info.position(); });
            
            
            odchylka = M1_pos - M4_pos; // otoceni 1 a 4
            integral += odchylka;

        man.motor(rb::MotorId::M1).power(i);
        man.motor(rb::MotorId::M4).power(-i  - odchylka * P - integral * I - (odchylka - last_odchylka) * D);
        Serial.printf("##### i+odchylka*P = %d | a = %d | M1_pos = %d | M4_pos = %d #####\n", i + odchylka * P, a, M1_pos, M4_pos);

        delay(20);
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;
        last_odchylka = odchylka;
    }
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    odchylka=0,integral=0,last_odchylka=0;
    jedu_pro_puk(true, M1_pos/3.7);
    Serial.println("MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");
    odchylka = 0, integral = 0;
}

void get_data(UARTResult_t vstup)
{
    while (1)
    {
        if (uart_recive(vstup))
        {
            memcpy(&uart_data, &vstup, sizeof(vstup));
        }
        else
        {
            vstup = {0};
        }
        delay(10);
    }
}

void get_senzor_data()
{
    while (1)
    {
        //! nastavení barevného senzoru !//
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
        tcs.getRGB(&senzor_data.r, &senzor_data.g, &senzor_data.b);
        //! nastavení barevného senzoru !//

        // dalkové senzory
        sensor1.rangingTest(&senzor_data.m1, false);
        sensor2.rangingTest(&senzor_data.m2, false);
        delay(10);
    }
}

//! Skoužka serv před začátkem
void servo_dance()
{
    auto &man = rb::Manager::get(); // vytvoří referenci na man class
    // Serial.begin(115200);
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
}

// točí se na základě času
void tocka(int smer, int cas) // smer 1 -> pravo -1 -> levo
{
    int target = 20000;
    int a = 2000;
    auto &man = rb::Manager::get(); // vytvoří referenci na man class
    for (int i = 0; i < target - 1; i += a)
    {

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info) {

        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info) {

        });
        man.motor(rb::MotorId::M1).power(i * smer);
        man.motor(rb::MotorId::M4).power(i * smer);
        delay(10);
    }
    // m1 musí být -
    // int M1_pos, M4_pos, odchylka, integral, last_odchylka =0;
    // int target = 10000;
    // int a = 500;

    for (int i = 0; i < cas; i += 1)
    {
        man.motor(rb::MotorId::M1).power(target * smer);
        man.motor(rb::MotorId::M4).power(target * smer); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               {
                                                   // M1_pos = -info.position();
                                               });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               {
                                                   // M4_pos = info.position();
                                               });
        // std::cout<<"odchylak: "<<M1_pos-M4_pos<<std::endl;

        delay(cas);
    }
    for (int i = 0; i > target; i -= a)
    {
        man.motor(rb::MotorId::M1).power(i * smer);
        man.motor(rb::MotorId::M4).power(i * smer);
        delay(10);
    }
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
}

//! najede na stěnu
// nutnost upravit tneto kod protože jsem zpravil trun funkci
void stena()
{
    auto &man = rb::Manager::get();                                            // vytvoří referenci na man class
    micros();                                                                  // update overflow
    int M1_pos = 0, M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0; // součet dvou ramp
    int P = 55, I = 0.01, D = 0.25;
    int target = 10000;
    int a = 500;

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);

    //! zrychlení
    for (int i = 0; i < target - 1; i += a)
    {
        Serial.print("Senzor 1 (0x30): ");
        Serial.print((senzor_data.m1.RangeStatus != 4) ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");
        Serial.print("Senzor 2 (0x31): ");
        Serial.print((senzor_data.m2.RangeStatus != 4) ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.printf("motor 1: %d | motor 4: %d \n", M1_pos, M4_pos);
        if ((senzor_data.m1.RangeMilliMeter < 210) || (senzor_data.m2.RangeMilliMeter < 300))
        {
            // stop =0;
            break;
        }
        odchylka = M1_pos - M4_pos;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });
        man.motor(rb::MotorId::M1).power(i + odchylka * 110);
        man.motor(rb::MotorId::M4).power(-i);
        delay(10);
        Serial.printf("M1: %d | M4 4: %d \n", M1_pos, M4_pos);
        //! musím zjistit jakou vzdálenost tímto ujedu nasledně ji z dvojnasobit a odečíst do požadované vzdálenosti
    }
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    while (!(senzor_data.m1.RangeMilliMeter < 210) && !(senzor_data.m2.RangeMilliMeter < 300))
    {
        Serial.print("Senzor 1 (0x30): ");
        Serial.print((senzor_data.m1.RangeStatus != 4) ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");
        Serial.print("Senzor 2 (0x31): ");
        Serial.print((senzor_data.m2.RangeStatus != 4) ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.printf("motor 1: %d | motor 4: %d \n", M1_pos, M4_pos);

        odchylka = M1_pos - M4_pos; // otoceni 1 a 4
        integral += odchylka;

        man.motor(rb::MotorId::M1).power(target + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        man.motor(rb::MotorId::M4).power(target * -1); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               {
                                                   M1_pos = info.position();
                                               });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });

        delay(50);
        std::cout << "M1: " << M1_pos << " M4: " << M4_pos << std::endl;
        last_odchylka = odchylka;
    }
    odchylka = 0, integral = 0;
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    //! doje ke stěně a jedne ze senzoru míří na stěnu

    while (senzor_data.m2.RangeMilliMeter < 460)
    {
        man.motor(rb::MotorId::M1).power(target);
        man.motor(rb::MotorId::M4).power(target);
        delay(50);
    }

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    man.motor(rb::MotorId::M1).power(27000); //! levý motor
    man.motor(rb::MotorId::M4).power(-32000);
    delay(1500);
}

void cesta_zpet()
{
    auto &man = rb::Manager::get(); // get manager instance as singleton
    stena();
    Serial.printf("red: %f, green: %f, blue: %f", senzor_data.r, senzor_data.g, senzor_data.b);
    Serial.print("######################\n");

    Serial.print("Senzor 1 (0x30): ");
    Serial.print((senzor_data.m1.RangeStatus != 4) ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    Serial.print("Senzor 2 (0x31): ");
    Serial.print((senzor_data.m2.RangeStatus != 4) ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    while (senzor_data.m2.RangeMilliMeter > 350)
    {
        Serial.printf("red: %f, green: %f, blue: %f", senzor_data.r, senzor_data.g, senzor_data.b);
        Serial.print("######################\n");

        Serial.print("Senzor 1 (0x30): ");
        Serial.print((senzor_data.m1.RangeStatus != 4) ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");
        Serial.print("Senzor 2 (0x31): ");
        Serial.print((senzor_data.m2.RangeStatus != 4) ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
        Serial.print("\n");

        man.motor(rb::MotorId::M1).power(27000); //! levý motor
        man.motor(rb::MotorId::M4).power(-32000);
        delay(50);
    }

    man.motor(rb::MotorId::M1).brake(27000); //! levý motor
    man.motor(rb::MotorId::M4).brake(32000);
    delay(1000);

    //! otevření kufru
    man.stupidServo(1).setPosition(-2.0f);
    delay(2000);
    //! popojezd v místě konce
    // turn(50,10000);
    toceni_dle_uhlu(70, 20000);

    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

    //! odjezd z pole
    for (int i = 0; i < 100; i++)
    {
        man.motor(rb::MotorId::M1).power(10000);
        man.motor(rb::MotorId::M4).power(-10000);
        delay(20);
    }
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    man.stupidServo(1).setPosition(-2.0f);
    delay(3000);

    //! zavření kufru
    man.stupidServo(1).setPosition(-0.1f); // -5 -90 -5
    delay(300000);
    //}
}
// inicializace senzorů
void set_up_peripherals()
{
    // start_time=millis();
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
    // delay(100);
    if (!sensor2.begin())
    {
        Serial.println("Nepodařilo se spustit senzor 2");
        while (1)
            ;
    }
    delay(100);
    scan_i2c();

    // writeRegister(0x30,0x8A,0x31);
    sensor2.setAddress(0x31);
    scan_i2c();

    //! senzor 1 inicializace
    Serial.println("zacina inicializace senzoru 1");
    digitalWrite(XSHUT1, HIGH);
    if (!sensor1.begin())
    {
        Serial.println("Nepodařilo se spustit senzor 1");
        while (1)
            ;
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

    tcs.begin(0x29, &Wire1);
    if (!tcs.begin(0x29, &Wire1))
    {
        Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
        while (1)
            ;
    }
    else
    {
        Serial.println("TCS34725 detekován.");
    }

    Serial.println("---- VSE OK ----");
}
// Hledání během otáčení



int hledani_90(int rychlost)
{
    int angle =90;
    bool puk_zpatren = false;
    int M1_pos = 0, M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0; //  rampa_vzdalenost = 640; 
    int M1_pred = 0, M4_pred = 0;
    float P = 4;
    float I = 0.001, D = 0.75;
    // int P =110, I = 0.01, D =0.5;
    int M4_smer = 1, M1_smer =1, smer =1;
    int ramp_distance = 0, a = 2000;
    double cil = 0;
    cil = ((PI * roztec) / (360 / abs(angle))) * 3.7; // roztec a kolo jsou v mm 

    if (angle < 0)  //M1 stoupá
    {
        smer = -1;
    }
    else            //M4 stoupá
    {
        smer = 1;
    }

    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    delay(100);
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });
    delay(100); 

    for (int i = 0; i < rychlost; i += 2000)
    {
        if (stopper)
            STOP();      

        if (vidim_puk(side, uart_data))
        {
            puk_zpatren=true;
            break;
        }

        if(!(smer)&&(M1_pos>0))
        {
            M1_smer=-1;
        }
        
        else 
            M1_smer = 1;

        if(smer&&(M4_pos>0))
        {
            M4_smer=-1;
        }
        else
            M4_smer =1;
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });

        odchylka = abs(abs(M4_pred)-M4_smer*abs(M4_pos))  - abs(abs(M4_pred)-M4_smer*abs(M4_pos)); //!odchylka = (M1_pos-M1_pred) + (M4_pos-M4_pred);
        integral += odchylka;

        man.motor(rb::MotorId::M1).power(i + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        man.motor(rb::MotorId::M4).power(i + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        Serial.printf("##### i+odchylka*P = %d | a = %d | M1_pos = %d | M4_pos = %d #####\n", i + odchylka * P, a, M1_pos, M4_pos);
        last_odchylka = odchylka;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;

        delay(20);
    }
    ramp_distance = M1_pos - M1_pred;
    while ((abs(abs(M1_pred) - M1_smer*abs(M1_pos)) < cil-ramp_distance) || (abs(abs(M4_pred)-M4_smer*abs(M4_pos)) < cil-ramp_distance)) //! odstranění cil - ramp_distance
    {
        if(puk_zpatren) break;
        if (vidim_puk(side, uart_data))
        {
            puk_zpatren = true;
            break;
        }
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });
        delay(50);
        if(!(smer)&&(M1_pos>0))
        {
            M1_smer=-1;
        }
        
        else 
            M1_smer = 1;

        if(smer&&(M4_pos>0))
        {
            M4_smer=-1;
        }
        else
            M4_smer =1;
        odchylka = abs(abs(M1_pred) - M1_smer*abs(M1_pos)) - abs(abs(M4_pred)-M4_smer*abs(M4_pos)) ; //!odchylka = (M1_pos-M1_pred) + (M4_pos-M4_pred);

        //!odchylka = M1_pos + M4_pos; // otoceni 1 a 4
        integral += odchylka;
        // man.motor(rb::MotorId::M1).setCurrentPosition(0);
        // man.motor(rb::MotorId::M4).setCurrentPosition(0);


        if ((abs(abs(M1_pred) - M1_smer*abs(M1_pos)) <= cil-ramp_distance))//! odstranění cil - ramp_distance
        {
            man.motor(rb::MotorId::M1).power(smer * rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        }
        else man.motor(rb::MotorId::M1).power(0);
        if ((abs(abs(M4_pred) - M4_smer*abs(M4_pos)) <= cil-ramp_distance))//! odstranění cil - ramp_distance
        {
            man.motor(rb::MotorId::M4).power(smer * rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        }
        else man.motor(rb::MotorId::M4).power(0);
        //! získá encodery z motoru
        // std::cout<<"cil: "<<abs(cil+M1_pred)<<" M1pos: "<<-1*M1_pos<<" M4 pos: "<<M4_pos<<std::endl;
        //! std::cout<<"odchylka: "<<M1_pos-M4_pos<<std::endl;
        Serial.printf("M1_pred = %d | M4_pred = %d | M1_pos = %d | M4_pos = %d | M1_pos-M1_pred = %d | Odchylka = %d | M4_pos-M4_pred = %d | speed = %f | smer = %d \n", M1_pred, M4_pred, M1_pos, M4_pos, abs(M1_pos - M1_pred), odchylka, abs(M4_pos - M4_pred), rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D,smer);
        last_odchylka = odchylka;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;

        delay(10);
        // last_odchylka = odchylka;
    }

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

    //! přenastavení počatečních hodnot
    odchylka = 0, integral = 0;
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });

    //! pokud je uhel věší jak 0 tak se přenastavý na novou hodnotu                                       
    if (abs(((M1_pos - M1_pred) * 360) / (3.7 * PI * roztec)) > 0)
        angle = abs(((M1_pos - M1_pred) * 360) / (3.7 * PI * roztec));

    Serial.printf("##### angle = %f #####", angle);
    // if (angle > 90)
        if (jedu_pro_puk(true, 200, angle, 1))
            return 1; //! něco přidatk angel nevím co

    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });

    Serial.printf("##### cil+M1_pred = %f #####\n", cil + M1_pred);
    Serial.printf("##### M1_pos-M1_pred = %f #####\n", M1_pos - M1_pred);

    //!#########################################################
    smer = -1;
    //angle = -90;

    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    delay(100);
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });
    delay(100); 

    for (int i = 0; i < rychlost; i += 2000)
    {
        if (stopper)
            STOP();      

        if (vidim_puk(side, uart_data))
        {
            puk_zpatren=true;
            break;
        }

        if(!(smer)&&(M1_pos>0))
        {
            M1_smer=-1;
        }
        
        else 
            M1_smer = 1;

        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });

        odchylka = abs(abs(M4_pred)-M4_smer*abs(M4_pos))  - abs(abs(M4_pred)-M4_smer*abs(M4_pos)); //!odchylka = (M1_pos-M1_pred) + (M4_pos-M4_pred);
        integral += odchylka;

        man.motor(rb::MotorId::M1).power(i + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        man.motor(rb::MotorId::M4).power(i + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        Serial.printf("##### i+odchylka*P = %d | a = %d | M1_pos = %d | M4_pos = %d #####\n", i + odchylka * P, a, M1_pos, M4_pos);
        last_odchylka = odchylka;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;

        delay(20);
    }
    //ramp_distance = 0;
    float multiplier = 2.25;
    ramp_distance = 0;
    while ((abs(abs(M1_pred) - M1_smer*abs(M1_pos)) < multiplier*cil-ramp_distance) || (abs(abs(M4_pred)-M4_smer*abs(M4_pos)) < 2*cil-ramp_distance)) //! odstranění cil - ramp_distance
    {
        if(puk_zpatren) break;
        if (vidim_puk(side, uart_data))
        {
            puk_zpatren = true;
            break;
        }
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });
        delay(50);
        if(!(smer)&&(M1_pos>0))
        {
            M1_smer=-1;
        }
        
        else 
            M1_smer = 1;


        odchylka = abs(abs(M1_pred) - M1_smer*abs(M1_pos)) - abs(abs(M4_pred)-M4_smer*abs(M4_pos)) ; //!odchylka = (M1_pos-M1_pred) + (M4_pos-M4_pred);

        //!odchylka = M1_pos + M4_pos; // otoceni 1 a 4
        integral += odchylka;
        // man.motor(rb::MotorId::M1).setCurrentPosition(0);
        // man.motor(rb::MotorId::M4).setCurrentPosition(0);


        if ((abs(abs(M1_pred) - M1_smer*abs(M1_pos)) <= multiplier*cil-ramp_distance))//! odstranění cil - ramp_distance
        {
            man.motor(rb::MotorId::M1).power(smer * rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        }
        else man.motor(rb::MotorId::M1).power(0);
        if ((abs(abs(M4_pred) - M4_smer*abs(M4_pos)) <= multiplier*cil-ramp_distance))//! odstranění cil - ramp_distance
        {
            man.motor(rb::MotorId::M4).power(smer * rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        }
        else man.motor(rb::MotorId::M4).power(0);
        //! získá encodery z motoru
        // std::cout<<"cil: "<<abs(cil+M1_pred)<<" M1pos: "<<-1*M1_pos<<" M4 pos: "<<M4_pos<<std::endl;
        //! std::cout<<"odchylka: "<<M1_pos-M4_pos<<std::endl;
        Serial.printf("M1_pred = %d | M4_pred = %d | M1_pos = %d | M4_pos = %d | M1_pos-M1_pred = %d | Odchylka = %d | M4_pos-M4_pred = %d | speed = %f | smer = %d \n", M1_pred, M4_pred, M1_pos, M4_pos, abs(M1_pos - M1_pred), odchylka, abs(M4_pos - M4_pred), rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D,smer);
        last_odchylka = odchylka;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;

        delay(10);
        // last_odchylka = odchylka;
    }

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

    //! přenastavení počatečních hodnot
    odchylka = 0, integral = 0;
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });

    //! pokud je uhel věší jak 0 tak se přenastavý na novou hodnotu                                       
    if (abs(((M1_pos - M1_pred) * 360) / (3.7 * PI * roztec)) > 0)
        angle = abs(((M1_pos - M1_pred) * 360) / (3.7 * PI * roztec));

    Serial.printf("##### angle = %f #####", angle);
    // if (angle > 90)
        if (jedu_pro_puk(true, 200, angle, 1))
            return 1; //! něco přidatk angel nevím co

    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });

    Serial.printf("##### cil+M1_pred = %f #####\n", cil + M1_pred);
    Serial.printf("##### M1_pos-M1_pred = %f #####\n", M1_pos - M1_pred);
    //!#########################################################

    smer = 1;
    
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    delay(100);
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });
    delay(100); 

    for (int i = 0; i < rychlost; i += 2000)
    {
        if (stopper)
            STOP();      

        if (vidim_puk(side, uart_data))
        {
            puk_zpatren=true;
            break;
        }

        if(smer&&(M4_pos>0))
        {
            M4_smer=-1;
        }
        else
            M4_smer =1;
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });

        odchylka = abs(abs(M4_pred)-M4_smer*abs(M4_pos))  - abs(abs(M4_pred)-M4_smer*abs(M4_pos)); //!odchylka = (M1_pos-M1_pred) + (M4_pos-M4_pred);
        integral += odchylka;

        man.motor(rb::MotorId::M1).power(i + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        man.motor(rb::MotorId::M4).power(i + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        Serial.printf("##### i+odchylka*P = %d | a = %d | M1_pos = %d | M4_pos = %d #####\n", i + odchylka * P, a, M1_pos, M4_pos);
        last_odchylka = odchylka;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;

        delay(20);
    }
    ramp_distance = M1_pos - M1_pred;
    while ((abs(abs(M1_pred) - M1_smer*abs(M1_pos)) < cil-ramp_distance) || (abs(abs(M4_pred)-M4_smer*abs(M4_pos)) < cil-ramp_distance)) //! odstranění cil - ramp_distance
    {
        if(puk_zpatren) break;
        if (vidim_puk(side, uart_data))
        {
            puk_zpatren = true;
            break;
        }
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                               { M1_pos = info.position(); });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                               { M4_pos = -info.position(); });
        delay(50);

        if(smer&&(M4_pos>0))
        {
            M4_smer=-1;
        }
        else
            M4_smer =1;
        odchylka = abs(abs(M1_pred) - M1_smer*abs(M1_pos)) - abs(abs(M4_pred)-M4_smer*abs(M4_pos)) ; //!odchylka = (M1_pos-M1_pred) + (M4_pos-M4_pred);

        //!odchylka = M1_pos + M4_pos; // otoceni 1 a 4
        integral += odchylka;

        if ((abs(abs(M1_pred) - M1_smer*abs(M1_pos)) <= cil-ramp_distance))//! odstranění cil - ramp_distance
        {
            man.motor(rb::MotorId::M1).power(smer * rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D);
        }
        else man.motor(rb::MotorId::M1).power(0);
        if ((abs(abs(M4_pred) - M4_smer*abs(M4_pos)) <= cil-ramp_distance))//! odstranění cil - ramp_distance
        {
            man.motor(rb::MotorId::M4).power(smer * rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D); // i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        }
        else man.motor(rb::MotorId::M4).power(0);
        //! získá encodery z motoru
        // std::cout<<"cil: "<<abs(cil+M1_pred)<<" M1pos: "<<-1*M1_pos<<" M4 pos: "<<M4_pos<<std::endl;
        //! std::cout<<"odchylka: "<<M1_pos-M4_pos<<std::endl;
        Serial.printf("M1_pred = %d | M4_pred = %d | M1_pos = %d | M4_pos = %d | M1_pos-M1_pred = %d | Odchylka = %d | M4_pos-M4_pred = %d | speed = %f | smer = %d \n", M1_pred, M4_pred, M1_pos, M4_pos, abs(M1_pos - M1_pred), odchylka, abs(M4_pos - M4_pred), rychlost + odchylka * P + integral * I + (odchylka - last_odchylka) * D,smer);
        last_odchylka = odchylka;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;

        delay(10);
        // last_odchylka = odchylka;
    }

    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);

    //! přenastavení počatečních hodnot
    odchylka = 0, integral = 0;
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });

    //! pokud je uhel věší jak 0 tak se přenastavý na novou hodnotu                                       
    if (abs(((M1_pos - M1_pred) * 360) / (3.7 * PI * roztec)) > 0)
        angle = abs(((M1_pos - M1_pred) * 360) / (3.7 * PI * roztec));

    Serial.printf("##### angle = %f #####", angle);
    // if (angle > 90)
        if (jedu_pro_puk(true, 200, angle, 1))
            return 1; //! něco přidatk angel nevím co

    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor &info)
                                           { M4_pred = -info.position(); });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor &info)
                                           { M1_pred = info.position(); });

    Serial.printf("##### cil+M1_pred = %f #####\n", cil + M1_pred);
    Serial.printf("##### M1_pos-M1_pred = %f #####\n", M1_pos - M1_pred);



}

// kod na homologaci
void homologace()
{
    // auto& man = rb::Manager::get(); // get manager instance as singleton

    hledani_vpred(100, 20000);
    delay(3000);

    // hledani_90(5500,1.0);
    cesta_zpet();

    if (stopper)
        STOP();
    delay(180000);
}
// otáčení dle zadaného uhlu -. přesnost do několika °

void setup()
{
    Serial.begin(115200);
    //     printf("Init manager instance\n");
    // auto& man = rb::Manager::get(); // get manager instance as singleton

    Serial.println("START");
    // rkConfig cfg;
    // rkSetup(cfg);
    Serial.println("RK hotovo");

    Serial.begin(115200);
    while (!Serial)
    {
        delay(1); //! dat vedet, co se stalo
    }

    // Wire.begin(21, 22);  // SDA = GPIO21, SCL = GPIO22
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
    // scan_i2c();

    Serial.println("zacina inicializace senzoru 2 #############");
    delay(100);
    if (!sensor2.begin())
    {
        Serial.println("Nepodařilo se spustit senzor 2");
        while (1)
            ;
    }
    delay(100);
    scan_i2c();

    // writeRegister(0x30,0x8A,0x31);
    sensor2.setAddress(0x31);
    scan_i2c();

    Serial.println("zacina inicializace senzoru 1 #############");
    digitalWrite(XSHUT1, HIGH);
    if (!sensor1.begin())
    {
        Serial.println("Nepodařilo se spustit senzor 1");
        while (1)
            ;
    }
    Serial.println("Podařilo se spustit senzor 1");
    scan_i2c();
    sensor1.setAddress(0x30);
    // writeRegister(0x29,0x8A,0x30);
    scan_i2c();

    pinMode(TCS_SDA_pin, PULLUP);
    pinMode(TCS_SCL_pin, PULLUP);
    Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro barevny senzor

    //! tcs.begin(0x29,&Wire1);
    //! if (!tcs.begin(0x29,&Wire1)) {
    //!     Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
    //!     while (1);
    //! } else {
    //!     Serial.println("TCS34725 detekován.");
    //! }
    scan_i2c();

    Serial.println("RB3204-RBCX\n");
    printf("RB3204-RBCX\n");
    delay(50);
    printf("Init manager\n");

    // auto& man = rb::Manager::get(); // get manager instance as singleton
    Serial.println("instance done");
    man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    Serial.println("stáhnuto done");

    micros(); // update overflow
    Serial.println("micros done");
    man.stupidServo(1).setPosition(-0.1f); // -5 -90 -5
    delay(1000);
    delay(200);

    uart_set_up();

    delay(200);

    std::thread uart_thread(get_data, uart_data);
    uart_thread.detach();

    std::thread i2c_thread(get_senzor_data);
    i2c_thread.detach();

    //! serva prozatim vypnuta
    // std::thread servo_thread(auto_servo);
    // servo_thread.detach();

    std::thread detekce_thread(detekce_nepritele, std::ref(stopper));
    detekce_thread.detach();
    delay(1000);
    Serial.println("start");
}
void loop()
{
    // auto& man = rb::Manager::get(); // get manager instance as singleton
    Serial.printf("red: %f, green: %f, blue: %f", senzor_data.r, senzor_data.g, senzor_data.b);
    // Serial.print("######################\n");

      toceni_dle_uhlu(90,12000);
      toceni_dle_uhlu(-90,12000);
      delay(1000);
    // homologace();
    //jizda_vpred(500,15000);
    hledani_90(12000);
    //hledani_vpred(300,15000);

    // Serial.print("######################\n");
    // delay(4800);
     //hledani_vpred(200,16000);
    // Serial.print("######################\n");
     //hledani_90(12000);
    //toceni_dle_uhlu(180, 8000);
    // delay(1200);
    // toceni_dle_uhlu(-90,12000);
    // hledani_90(14000);

    Serial.print("######################\n");
    // jedu_pro_puk(true,200);
    // Serial.print("######################\n");

    // jizda_vpred(200,8000);
    // hledani_vpred(200,8000);
    // hledani_90(7000);
    delay(1000000);
}
