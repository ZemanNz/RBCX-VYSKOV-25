#include "robotka.h"

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