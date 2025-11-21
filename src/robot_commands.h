#include "robotka.h"

int8_t posledni_natoceni = 1;
const int in1 =  25;
const int in2 =  26;
const int in3 =  27;
const int in4 =  14;
int rychlost = 1; // mensi cislo = nizsi rychlost

void otevri_zasobnik(int8_t id){
    rkServosSetPosition(id, -70);
}

void zavri_zasobnik(int8_t id){
    rkServosSetPosition(id, 85);
}

void otevri_vsechny_zasobniky(){
    rkServosSetPosition(2, -70);
    rkServosSetPosition(3, -70);
    rkServosSetPosition(4, -70);
}

void zavri_vsechny_zasobniky(){
    rkServosSetPosition(2, 85);
    rkServosSetPosition(3, 80);
    rkServosSetPosition(4, 85);
}

void init_ruka(){
    rkSmartServoInit(0, 3, 190, 500, 3);
    rkSmartServoInit(1, 200, 229, 500, 3);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void otevri_klepata(){
    rkSmartServoMove(1,200, 300);
}

void zavri_klepeta(){
    //rkSmartServoSoftMove(1,228,100); /// prvni test
    rkSmartServoSoftMove(1,240,100); // test doma drevo
}

void ruka_nahoru(){
    rkSmartServoMove(0,185, 400);
}

void ruka_dolu(){
    rkSmartServoMove(0,9, 300);
}


void krok1(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(rychlost);
}
void krok2(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(rychlost);
}
void krok3(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(rychlost);
}
void krok4(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(rychlost);
}
void krok5(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(rychlost);
}
void krok6(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  delay(rychlost);
}
void krok7(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(rychlost);
}
void krok8(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(rychlost);
}

void rotacePoSmeru() {
  krok1();
  krok2();
  krok3();
  krok4();
  krok5();
  krok6();
  krok7();
  krok8();
}
void rotaceProtiSmeru() {
  krok8();
  krok7();
  krok6();
  krok5();
  krok4();
  krok3();
  krok2();
  krok1();
}

void otoc_motorem(int uhel, bool ruka_po_smeru){
  int pocet_kroku = (uhel * 64) / 45;
  if(ruka_po_smeru){
    for(int i=0;i<pocet_kroku;i++){
      rotaceProtiSmeru();
    }
  }
  else{
    for(int i=0;i<pocet_kroku;i++){
      rotacePoSmeru();
    }
  }

}

void natocit_ruku(int8_t id_zasobniku){ // // R - 0, G - 1 , B - 2
    if(posledni_natoceni == id_zasobniku){
        return;
    }
    int uhel = 0;
    bool smer = true; // true - po smeru, false - proti smeru
    if(id_zasobniku == 1){//4 moznosti.....
        if(posledni_natoceni == 0){ // otaceni k nabirani
            smer = false;
            uhel = 250;
        }
        else{
            smer = true;
            uhel = 240;
        }
    }
    else if(id_zasobniku == 0){// otaceni k vhazovani
        smer = true;
        uhel = 250;
    }
    else if(id_zasobniku == 2){
        smer = false;
        uhel = 240;
    }
    else{
        return;
    }
    posledni_natoceni = id_zasobniku;
    otoc_motorem(uhel, smer);
    }

void chyt_a_uloz_kostku(){
    zavri_klepeta();
    delay(500);
    int8_t id_zasobniku = 5; // R - 0, G - 1 , B - 2
    float r,g,b;
    rkColorSensorGetRGB("klepeta_senzor", &r, &g, &b); // je potreba inicializovat v setup
    Serial.print(" R: "); Serial.print(r, 3);
    Serial.print(" G: "); Serial.print(g, 3);
    Serial.print(" B: "); Serial.println(b, 3);

    delay(300);
    if(r > g && r > b && r > 140){ // tohle jeste dodelatna zakladenamerenych hodnot...
        id_zasobniku = 0; // cervena je 0
    }
    if(g > r && g> b && g > 90){
        id_zasobniku = 1; // zelena je 1
    }
    if(b > g && b> r && b > 100){
        id_zasobniku = 2; // modra je 2
    }
    std::cout<< "Zasobnik ID: " << (int)id_zasobniku << std::endl;

    if(id_zasobniku < 0 || id_zasobniku >2){
        otevri_klepata();
        return;
    }
    ruka_nahoru();
    delay(500);

    natocit_ruku(id_zasobniku);

    delay(400);

    otevri_klepata();
    delay(500);

    natocit_ruku(1); // 1 je na nabirani a G

    delay(800);

    ruka_dolu();
    delay(400);
}

bool try_to_catch(){
    rkSmartServoSoftMove(1,220,100);
    delay(100);
    float r,g,b;
    rkColorSensorGetRGB("klepeta_senzor", &r, &g, &b); // je potreba inicializovat v setup
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