#include "robotka.h"

int8_t posledni_natoceni = 1;
const int in1 =  25;
const int in2 =  26;
const int in3 =  27;
const int in4 =  14;
int rychlost = 1; // vetsi cislo = nizsi rychlost



/////////////////////////////////////////////////////////////////////////////////////nastavovani rgb
float r,g,b;

bool prazdno(){
    if( (r > 210 && g > 210 && b > 210)){
        return true;
    }
    return false;
}

bool is_red_detected(){
    if((r > (g + 15)) && (r > (b +10))  && (r > 130)){
        return true;
    }
    return false;
}
bool is_green_detected(){
    if((g > (r + 11)) && (g> (b + 10)) && (g > 100)){
        return true;
    }
    return false;
}

bool is_blue_detected (){
    if((b > (g + 15)) && (b> (r + 15)) && (b > 110)){
        return true;
    }
    return false;
}
    
bool is_still_red_detected(){
    if(r > g && r > b && r > 110){
        return true;
    }
    return false;
}

bool is_still_green_detected(){
    if((g > r && g > b && g > 100)){
        return true;
    }
    return false;
}

bool is_still_blue_detected(){
    if(b > g && b > r && b > 100){
        return true;
    }
    return false;
}


/////////////////////////////////////////////////////////////////////////////////////




void otevri_zasobnik(int8_t id){
    rkServosSetPosition(id, -70);
}

void zavri_zasobnik(int8_t id){
    rkServosSetPosition(id, 85);
}

void otevri_vsechny_zasobniky(){
    rkServosSetPosition(2, 0);
    rkServosSetPosition(3, 0);// zelezny
    rkServosSetPosition(4, 0);
    delay(300);
    rkServosSetPosition(2, -40);
    rkServosSetPosition(3, -40);// zelezny
    rkServosSetPosition(4, -40);
    delay(300);
    rkServosSetPosition(2, -70);
    rkServosSetPosition(3, -70);// zelezny
    rkServosSetPosition(4, -70);
}

void zavri_vsechny_zasobniky(){
    rkServosSetPosition(2, 90);
    rkServosSetPosition(3, 72);
    rkServosSetPosition(4, 85);
}

void init_ruka(){
    rkSmartServoInit(0, 3, 190, 500, 3);
    rkSmartServoInit(1, 160, 200, 120, 8); // rkSmartServoInit(1, 160, 200, 350, 4);// 8, 120
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void otevri_klepeta(){
    rkSmartServoMove(1,160, 300);
}

void zavri_klepeta(){
    rkSmartServoSoftMove(1,200,200);
    // while(rkSmartServosPosicion(1) < 195){
    //     std::cout<< "Klepeta pozice: " << (int)rkSmartServosPosicion(1) << std::endl;
    //     delay(10);
    // }
}

// jeste dodela funkce na mereni barev
void measure_color(){
    while(true){
        zavri_klepeta();
        delay(1000);
        rkColorSensorGetRGB("klepeta_senzor", &r, &g, &b);
        delay(100);
        std::cout<< " Mereni barev - R: " << r << " G: " << g << " B: " << b << std::endl;
        otevri_klepeta();
        delay(4000);
    }
}

bool je_kostka_v_klepete(){
    rkSmartServoSoftMove(1,200,200);
    delay(300);
    std::cout<< "Klepeta pozice po dozavreni: " << (int)rkSmartServosPosicion(1) << std::endl;
    if(rkSmartServosPosicion(1) < 196){
        return true;
    }
    return false;
}

void ruka_nahoru(){
    rkSmartServoMove(0,185, 600);
    while(rkSmartServosPosicion(0) < 180){
        rkSmartServoMove(0,185, 600);
        delay(10);
    }
}

void ruka_top_nahoru(){
    rkSmartServoMove(0,170, 600);
    while(rkSmartServosPosicion(0) < 165){
        rkSmartServoMove(0,170, 600);
        delay(10);
    }
}

void ruka_dolu(){
    rkSmartServoMove(0,9, 400);
    while(rkSmartServosPosicion(0) > 10){
        rkSmartServoMove(0,9, 400);
        delay(10);
    }
}

void ruka_na_kontrolu(){
    rkSmartServoMove(0,40, 600);
    while(rkSmartServosPosicion(0) < 35){
        rkSmartServoMove(0,40, 600);
        delay(10);
    }
}

void ruka_nahoru_neb(){
    rkSmartServoMove(0,185, 600);
}

void ruka_top_nahoru_neb(){
    rkSmartServoMove(0,170, 600);
}

void ruka_dolu_neb(){
    rkSmartServoMove(0,9, 400);
}

bool je_tam_kostka_ir(){
    int r_ir = rkIrRight();
    std::cout<< " IR pravy: " << r_ir << std::endl;
    if(r_ir < 150 && r_ir > 10){
        return true;
    }
    return false;
}

void zavri_prepazku(){
  rkServosSetPosition(1, -70);
}
void otevri_prepazku(){
  rkServosSetPosition(1, 40);
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

void nastav_ruku_na_start(){
  zavri_klepeta();
  delay(200);
  ruka_dolu();
  otoc_motorem(100, false);
  delay(500);
  otoc_motorem(62, true);
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
            uhel = 243;
        }
        else{
            smer = true;
            uhel = 240;
        }
    }
    else if(id_zasobniku == 0){// otaceni k vhazovani
        smer = true;
        uhel = 243;
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

bool mame_porad_kostku(char barva){

    if(je_kostka_v_klepete()){
        std::cout<< "Kostka je v klepete, kontrola barvy" << std::endl;
        
        rkColorSensorGetRGB("klepeta_senzor", &r, &g, &b); // je potreba inicializovat v setup
        delay(50);

        std::cout<< " Kontrola kostky - R: " << r << " G: " << g << " B: " << b << std::endl;
        
        if(!(prazdno())){
            if(barva == 'R' && is_still_red_detected()){
                std::cout<< "Porad mame kostku" << std::endl;
                return true;
            }
            if(barva == 'G' && is_still_green_detected()){
                std::cout<< "Porad mame kostku" << std::endl;
                return true;
            }
            if(barva == 'B' && is_still_blue_detected()){
                std::cout<< "Porad mame kostku" << std::endl;
                return true;
            }
        }
    }
    std::cout<< "Uz nemame kostku" << std::endl;
    return false;
}

void chyt_a_uloz_kostku(){
    int start_time = millis();
    zavri_klepeta();
    delay(500);
    int8_t id_zasobniku = 5; // R - 0, G - 1 , B - 2
    rkColorSensorGetRGB("klepeta_senzor", &r, &g, &b); // je potreba inicializovat v setup
    Serial.print(" R: "); Serial.print(r, 3);
    Serial.print(" G: "); Serial.print(g, 3);
    Serial.print(" B: "); Serial.println(b, 3);

    
    //printf_wifi(" R: %f G: %f B: %f\n", r, g, b);

    delay(100);

    //handleWebClients();

    if((prazdno())|| !(je_kostka_v_klepete())){ // na prazdno
        std::cout<< "Na prazdno" << std::endl;
        otevri_klepeta();
        delay(500);
        return;
    }

    std::cout<< "Jdem se roozhodovat " << std::endl;

    if(is_red_detected()){ // tohle jeste dodelatna zakladenamerenych hodnot...
        id_zasobniku = 0; // cervena je 0
        std::cout<< "Zasobnik ID: " << (int)id_zasobniku << std::endl;

        ruka_na_kontrolu();

        if(!mame_porad_kostku('R')){
            rkSmartServoMove(1,170, 300); // mirnr zavri klepeta
            ruka_dolu();
            otevri_klepeta();
            return;
        }

        ruka_top_nahoru_neb();

        natocit_ruku(id_zasobniku);

        ruka_nahoru();

        otevri_klepeta();
        delay(500);

        ruka_top_nahoru_neb();

        natocit_ruku(1); // 1 je na nabirani a G

        delay(100);

        rkSmartServoMove(1,170, 300); // mirnr zavri klepeta


        ruka_dolu();
        otevri_klepeta();
    }
    if(is_green_detected()){
        id_zasobniku = 1; // zelena je 1
        std::cout<< "Zasobnik ID: " << (int)id_zasobniku << std::endl;
        ruka_nahoru();
        otevri_klepeta();
        delay(700);
        rkSmartServoMove(1,170, 300); // mirnr zavri klepeta
        ruka_dolu();
        otevri_klepeta();
    }
    if(is_blue_detected()){
        id_zasobniku = 2; // modra je 2
        std::cout<< "Zasobnik ID: " << (int)id_zasobniku << std::endl;

        ruka_na_kontrolu();

        if(!mame_porad_kostku('B')){
            rkSmartServoMove(1,170, 300); // mirnr zavri klepeta
            ruka_dolu();
            otevri_klepeta();
            return;
        }

        ruka_top_nahoru_neb();
        // delay(100);

        natocit_ruku(id_zasobniku);

        ruka_nahoru();

        otevri_klepeta();
        delay(500);

        ruka_top_nahoru_neb();

        natocit_ruku(1); // 1 je na nabirani a G

        delay(100);
        rkSmartServoMove(1,170, 300); // mirnr zavri klepeta
        ruka_dolu();

        otevri_klepeta();

    }

    otevri_klepeta();
    delay(500);
}


bool try_to_catch(){
    zavri_klepeta();
    delay(600);
    rkColorSensorGetRGB("klepeta_senzor", &r, &g, &b); // je potreba inicializovat v setup
    delay(100);
    Serial.print(" R: "); Serial.print(r, 3);
    Serial.print(" G: "); Serial.print(g, 3);
    Serial.print(" B: "); Serial.println(b, 3);
    
    if(is_still_red_detected() || is_still_green_detected() || is_still_blue_detected()){

        return true;
    }
    otevri_klepeta();
    delay(10);
    return false;
}