#include "robotka.h"

void setup() {
    Serial.begin(115200);
    rkConfig cfg;
    rkSetup(cfg);

    // Nastavení serva
    rkServosSetPosition(1, 85); //85 stupnu je zavrene zasobniky
    rkServosSetPosition(2, 85);
    rkServosSetPosition(3, 85);
}

void loop() {

    if (rkButtonIsPressed(BTN_UP)) {
        rkServosSetPosition(1, -70); //-70 stupnu je otevrene zasobniky
    }
    if (rkButtonIsPressed(BTN_RIGHT)) {
        rkServosSetPosition(2, -70);
    }
    if (rkButtonIsPressed(BTN_LEFT)) {
        rkServosSetPosition(3, -70);
    }
}