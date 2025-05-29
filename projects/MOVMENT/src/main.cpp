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
// ((poloměr + rozteč)*4000) × π × stupně // v metrech
//#include "robotka.h"
UARTResult_t uart_data;
I2cout senzor_data;

// cíl v cm
// rychlost v rozmezí -32768 až 32768
void jizda_vpred(int cil,int rychlost)
{
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);


    //m1 musí být -
    int M1_pos = 0, M4_pos = 0, odhylaka = 0, integral = 0, last_odchylka =0, rampa_vzdalenost = 640;
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
        std::cout<<"ODchilka: "<<odhylaka<<std::endl;
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
void turn(int angle, int rychlost)
{
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    //m1 musí být -
    int M1_pos = 0, M4_pos = 0, odhylaka = 0, integral = 0, last_odchylka =0, cil = 0;
    int target = rychlost;
    int a = 500;
    cil = (roztec+r_kola)*40;

    while(-1*M1_pos<cil)   //! 4000 převod na metry
    {
        std::cout<<"cil: "<<cil<<"M1pos: "<<-1*M1_pos<<std::endl;
        man.motor(rb::MotorId::M1).setCurrentPosition(0);
        man.motor(rb::MotorId::M4).setCurrentPosition(0);
        odhylaka = M1_pos-M4_pos;
        integral += odhylaka; 

        man.motor(rb::MotorId::M1).power(20000+odhylaka*55+integral*0.01+(odhylaka-last_odchylka)*0.1);
        man.motor(rb::MotorId::M4).power(20000*-1);// i míň se to kvedla 50 -55 je ok  bez derivace )poslední čast na 2,5 m 3cm odchylka
        //! získá encodery z motoru
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        std::cout<<"odchylak: "<<M1_pos-M4_pos<<std::endl;

        delay(50);
        last_odchylka = odhylaka;
    }


    
}
void get_data(UARTResult_t vstup)
{
    while(1)
    {
       if( uart_recive(vstup))
       {
           memcpy(&uart_data,&vstup,sizeof (vstup));

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

    // dalkové senzory
     sensor1.rangingTest(&senzor_data.m1, false);
     sensor2.rangingTest(&senzor_data.m2, false);
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
    auto& man = rb::Manager::get(); // vytvoří referenci na man class
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

void setup() {
    //turn(1,1);
  start_time=millis();
  Serial.begin(115200);
  while (! Serial) {
    delay(1);
  }

  Wire1.begin(TCS_SDA_pin, TCS_SCL_pin, 100000); // pro predni senzor 
  //tcs.begin(0x29,&Wire1);
  if (!tcs.begin(0x29,&Wire1)) {
      Serial.println("Barevný senzor TCS34725 nebyl nalezen!");
      while (1);
    } else {
        Serial.println("TCS34725 detekován.");
    }
    // delay(10);
    // Serial.println("Wire (I2C) inicializováno na SDA=21, SCL=22");
    Wire.begin(21, 22);

    //Wire.begin(21, 22);  // SDA = GPIO21, SCL = GPIO22
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);

    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    delay(20);

    digitalWrite(XSHUT1, HIGH);
    delay(10);
    scan_i2c();
  
    if (!sensor1.begin()) {
      Serial.println("Nepodařilo se spustit senzor 1");
      while (1);
    }
    scan_i2c();
    sensor1.setAddress(0x30);
   // writeRegister(0x29,0x8A,0x30);
      scan_i2c();

    digitalWrite(XSHUT2, HIGH);
    delay(10);
    scan_i2c();
  
    Serial.println("zacina inicializace #############");
    if (!sensor2.begin()) {
      Serial.println("Nepodařilo se spustit senzor 2");
      while (1);
    }
    
    //scan_i2c();
  //writeRegister(0x30,0x8A,0x31);
  sensor2.setAddress(0x31);
  scan_i2c();


    Serial.println("############# zacina inicializace #############");
    //scan_i2c();    

  pinMode(TCS_SDA_pin, PULLUP);

  pinMode(TCS_SCL_pin, PULLUP);




    printf("RB3204-RBCX\n");
    delay(50);
    printf("Init manager\n");

    auto& man = rb::Manager::get(); // get manager instance as singleton
    man.install(rb::ManagerInstallFlags::MAN_DISABLE_MOTOR_FAILSAFE); // install manager
    micros(); // update overflow
    delay(200);

    //!uart_set_up();


    //delay(1000);


    //lox.begin();
    delay(200);

    writeRegister(VL53L0X_DEFAULT_ADDR, I2C_SLAVE_DEVICE_ADDRESS, VL53L0X_NEW_ADDR);
    delay(200);
    
    // Inicializace senzoru na nové adrese

    Serial.println("Senzor VL53L0X uspesne inicializovan.");
    tcs.begin(0x29);

    //uart_thread_function();
    std::thread uart_thread (get_data, uart_data);
    uart_thread.detach();

    std::thread i2c_thread(get_senzor_data);
    i2c_thread.detach();
    Serial.println("start");
    while(1)
    {
        if(man.buttons().right())
        {
            Serial.println("*******************************");
            delay(1000);
            turn(90,10000);
            //jizda_vpred(50,20000);
        }
        delay(200);
    }

}
void loop()
{
    // while(1)
    // {
   // if (man.buttons().down()) {
    //    break;
   // }
    // delay(100);
   // }
    if(millis()-start_time >=end_time)
    {
        while(1)
        {
           Serial.print("---- HOTOVO ----\n"); 
        }
    }
    //cervena();
    Serial.printf("red: %f, green: %f, blue: %f",senzor_data.r,senzor_data.g,senzor_data.b);
    Serial.print("######################\n");
    
    Serial.print("Senzor 1 (0x30): ");
    Serial.print((senzor_data.m1.RangeStatus != 4)  ? String(senzor_data.m1.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    Serial.print("Senzor 2 (0x31): ");
    Serial.print((senzor_data.m2.RangeStatus != 4)  ? String(senzor_data.m2.RangeMilliMeter) + " mm\t" : "Mimo rozsah\t");
    Serial.print("\n");
    delay(100);
    Serial.println("---- UART START ----");

    int x1 =0;
    int x2 =0;
    int center = 125;

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
                    if (object_center < 185 && object_center > 110) { //! Upraveno pro použití object_center
                        Serial.println("##### MAM TO #####");
                    }
                    else if (object_center > center) { //! Upraveno pro použití object_center
                                Serial.println("##### RRRRRRRRRRRRRRRRRRRRRRRRR #####");
                        //!tocka(1, 90);
                        delay(300);
                    }
                    else if (object_center < center && object_center > 30) { //! Upraveno pro použití object_center
                        Serial.println("##### LLLLLLLLLLLLLLLLLLL #####");
                        //!tocka(-1, 90);
                        delay(300);
                    }
                }
                else{
                    Serial.println("------------- je to modre -------------");
                }
                uart_data = {0};
        }
    }
      
}
    