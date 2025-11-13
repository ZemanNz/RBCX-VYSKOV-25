#include "robotka.h"

int8_t posledni_natoceni = 1;

void otevri_zasobnik(int8_t id){
    rkServosSetPosition(id, -70);
}

void zavri_zasobnik(int8_t id){
    rkServosSetPosition(id, 85);
}

void otevri_vsechny_zasobniky(){
    rkServosSetPosition(0, -70);
    rkServosSetPosition(1, -70);
    rkServosSetPosition(2, -70);
}

void zavri_vsechny_zasobniky(){
    rkServosSetPosition(0, 85);
    rkServosSetPosition(1, 85);
    rkServosSetPosition(2, 85);
}

void init_ruka(){
    rkSmartServoInit(0, 3, 190, 500, 3);
    rkSmartServoInit(1, 200, 229, 500, 3);
}

void otevri_klepata(){
    rkSmartServoMove(1,200, 100);
}

void zavri_klepeta(){
    rkSmartServoSoftMove(1,228,100);
}

void ruka_nahoru(){
    rkSmartServoMove(0,185, 100);
}

void ruka_dolu(){
    rkSmartServoMove(0,5, 100);
}

void natocit_ruku(int8_t id_zasobniku){ // // R - 0, G - 1 , B - 2
    if(posledni_natoceni == id_zasobniku){
        return;
    }
    int16_t pozice = 0;
    int16_t konecna_pozice = 800;
    auto& man = rb::Manager::get();
    int speed = 0;

    if(id_zasobniku == 1){//4 moznosti.....
        if(posledni_natoceni == 0){ // otaceni k nabirani
            speed = 10;
        }
        else{
            speed= -10;
        }
    }
    else if(id_zasobniku == 0){// otaceni k vhazovani
        speed = -10;
    }
    else if(id_zasobniku == 2){
        speed = 10;
    }
    man.motor(rb::MotorId (2)).setCurrentPosition(0);
    
    delay(5);

    man.motor(rb::MotorId (2)).speed(speed);

    while(abs(pozice) < konecna_pozice){
        delay(5);

        man.motor(rb::MotorId (2)).requestInfo([&](rb::Motor& info) {
             pozice = info.position();
          });
    }
    
    man.motor(rb::MotorId (2)).speed(0);    
}

void chyt_a_uloz_kostku(){
    zavri_klepeta();
    delay(100);
    int8_t id_zasobniku; // R - 0, G - 1 , B - 2
    float r,g,b;
    rkColorSensorGetRGB("klepeta", &r, &g, &b); // je potreba inicializovat v setup
    Serial.print(" R: "); Serial.print(r, 3);
    Serial.print(" G: "); Serial.print(g, 3);
    Serial.print(" B: "); Serial.println(b, 3);

    if(r > g && r > b){ // tohle jeste dodelatna zakladenamerenych hodnot...
        id_zasobniku = 0;
    }
    if(g > r && g> b){
        id_zasobniku = 1;
    }
    if(b > g && b> r){
        id_zasobniku = 2;
    }

    ruka_nahoru();
    delay(200);

    natocit_ruku(id_zasobniku);

    otevri_klepata();
    delay(100);

    natocit_ruku(1); // 1 je na nabirani a G

    ruka_dolu();
    delay(200);
}

bool try_to_catch(){
    rkSmartServoSoftMove(1,220,100);
    delay(100);
    float r,g,b;
    rkColorSensorGetRGB("klepeta", &r, &g, &b); // je potreba inicializovat v setup
    Serial.print(" R: "); Serial.print(r, 3);
    Serial.print(" G: "); Serial.print(g, 3);
    Serial.print(" B: "); Serial.println(b, 3);

    if(abs(r -g) > 80 || abs(g-b) > 80 || abs(r - b) > 80){
        delay(100);
        return true;
    }
    rkSmartServoMove(1,200, 100);
    delay(100);
    return false;
}