#include "robotka.h"
#include "robot_commands.h"
#include "robot_pohyb.h"

rb::MotorId left_id;
rb::MotorId right_id;
int max_speed;
float wheel_circumference;
float prevod_motoru;
float wheel_circumference_left;
float wheel_circumference_right;
bool polarity_switch_left;
bool polarity_switch_right;
byte Button1;
byte Button2;

Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
float r1, g1, b1;

TaskHandle_t taskChytHandle = NULL;
SemaphoreHandle_t chytaniSemafor = NULL;


// task spouští chyt_a_uloz_kostku a po skončení se ukončí a vyčistí handle
void taskChytAUlozKostku(void *pvParameters) {
    chyt_a_uloz_kostku();

    // signalizace skončení
    taskChytHandle = NULL;
    xSemaphoreGive(chytaniSemafor); // Uvolnění zámku, aby mohl běžet další task

    vTaskDelete(NULL); // smaže sebe
}



void nachystej_se(){
    rkLedBlue(true);

    zavri_vsechny_zasobniky();
    ruka_dolu();
    delay(100);
    nastav_ruku_na_start();
    otevri_klepeta();
    otevri_prepazku();
    
    rkLedBlue(false);

}

int dozadu_dlouze = 150;
int dozadu_kratce = 120;

// ============================================================================
// TRASY S GYROSKOPEM, GLOBÁLNÍM ÚHLEM (0° -> 270° -> 180° -> 90° -> 0°)
// A NULOVÁNÍM GYRA POUZE O TLAČÍTKA (dorovnávání ultrazvukem orient_to_wall vypnuto)
// ============================================================================

void vyhrej_to_rychlejsi_gyro() {
    rkLedGreen(true);
    bool sbirej = true;
    int start_time = millis();

    // Výchozí globální úhel na startu = 0°
    gyroReset();

    ///////////////////////////////////// prvni (start 0° -> otočení na 270° -> 180°)
    jed_a_sbirej_kostky_buttons_fast_gyro(0.0f);
    backward_gyro(dozadu_kratce, 30, 0.0f);
    turn_gyro(270.0f, 20); // 1. otočení doleva: globální úhel = 270° (-90°)
    delay(100);

    jed_a_sbirej_kostky_buttons_gyro(270.0f);
    backward_gyro(dozadu_dlouze, 30, 270.0f);
    turn_gyro(180.0f, 20); // 2. otočení doleva: globální úhel = 180°
    delay(100);
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// druha (180° -> otočení na 90°)
    jed_a_sbirej_kostky_buttons_gyro(180.0f);
    backward_gyro(dozadu_kratce, 30, 180.0f);
    turn_gyro(90.0f, 20); // 3. otočení doleva: globální úhel = 90°
    delay(100);
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// treti (90° -> otočení na 0°)
    jed_a_sbirej_kostky_buttons_fast_gyro(90.0f);
    backward_gyro(dozadu_dlouze, 30, 90.0f);
    int vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);

    if (vzdalenost_od_zdi < 90) { // do toho vzdáleného rohu
        turn_gyro(120.0f, 20);    // 30° doprava z 90° -> 120°
        backward_gyro(dozadu_dlouze, 30, 120.0f);
        turn_gyro(90.0f, 20);     // 30° doleva zpět na 90°
        delay(100);
        jed_a_sbirej_kostky_buttons_fast_gyro(90.0f);
        backward_gyro(150, 30, 90.0f);
    }

    turn_gyro(0.0f, 20); // 4. otočení doleva: globální úhel = 0°
    delay(100);
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// ctvrta (0° -> otočení na 270°)
    jed_a_sbirej_kostky_buttons_fast_gyro(0.0f);
    backward_gyro(dozadu_kratce, 30, 0.0f);
    vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);
    turn_gyro(270.0f, 20); // 5. otočení doleva: globální úhel = 270°
    delay(100);
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// pata (270° -> otočení na 180°)
    int time_od_startu = millis() - start_time;
    std::cout << "Cas od startu: " << time_od_startu << " ms" << std::endl;

    if (time_od_startu > 150000) {
        std::cout << "Musim rychle dojet" << std::endl;
        sbirej = false;
        rkLedRed(true);
    }

    int draha = 880 - vzdalenost_od_zdi;
    jed_a_sbirej_kostky_mm_gyro(draha, sbirej, 270.0f);
    delay(100);
    vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);
    turn_gyro(180.0f, 20); // 6. otočení doleva: globální úhel = 180°
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// sesta (180°)
    draha = 470 - vzdalenost_od_zdi;
    jed_a_sbirej_kostky_mm_gyro(draha, sbirej, 180.0f);
    otevri_vsechny_zasobniky();
    forward_gyro(240, 15, 180.0f);
    delay(500);
    zavri_vsechny_zasobniky();
    rkLedGreen(false);
    rkLedRed(false);
}


void vyhrej_to_gyro() {
    rkLedGreen(true);
    bool sbirej = true;
    int start_time = millis();

    // Výchozí globální úhel na startu = 0°
    gyroReset();

    ///////////////////////////////////// prvni (start 0° -> otočení na 270° -> 180°)
    delay(1000);
    jed_a_sbirej_kostky_buttons_gyro(0.0f);
    backward_gyro(dozadu_kratce, 30, 0.0f);
    turn_gyro(270.0f, 20); // 1. otočení doleva: globální úhel = 270° (-90°)
    delay(100);

    jed_a_sbirej_kostky_buttons_gyro(270.0f);
    backward_gyro(dozadu_dlouze, 30, 270.0f);
    turn_gyro(180.0f, 20); // 2. otočení doleva: globální úhel = 180°
    delay(100);
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// druha (180° -> otočení na 90°)
    jed_a_sbirej_kostky_buttons_gyro(180.0f);
    backward_gyro(dozadu_kratce, 30, 180.0f);
    turn_gyro(90.0f, 20); // 3. otočení doleva: globální úhel = 90°
    delay(100);
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// treti (90° -> otočení na 0°)
    jed_a_sbirej_kostky_buttons_gyro(90.0f);
    backward_gyro(dozadu_dlouze, 30, 90.0f);
    int vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);

    if (vzdalenost_od_zdi < 90) { // do toho vzdáleného rohu
        turn_gyro(120.0f, 20);    // 30° doprava z 90° -> 120°
        backward_gyro(dozadu_dlouze, 30, 120.0f);
        turn_gyro(90.0f, 20);     // 30° doleva zpět na 90°
        delay(100);
        jed_a_sbirej_kostky_buttons_gyro(90.0f);
        backward_gyro(150, 30, 90.0f);
    }

    turn_gyro(0.0f, 20); // 4. otočení doleva: globální úhel = 0°
    delay(100);
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// ctvrta (0° -> otočení na 270°)
    jed_a_sbirej_kostky_buttons_gyro(0.0f);
    backward_gyro(dozadu_kratce, 30, 0.0f);
    vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);
    turn_gyro(270.0f, 20); // 5. otočení doleva: globální úhel = 270°
    delay(100);
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// pata (270° -> otočení na 180°)
    int time_od_startu = millis() - start_time;
    std::cout << "Cas od startu: " << time_od_startu << " ms" << std::endl;

    if (time_od_startu > 150000) {
        std::cout << "Musim rychle dojet" << std::endl;
        sbirej = false;
        rkLedRed(true);
    }

    int draha = 880 - vzdalenost_od_zdi;
    jed_a_sbirej_kostky_mm_gyro(draha, sbirej, 270.0f);
    delay(100);
    vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);
    turn_gyro(180.0f, 20); // 6. otočení doleva: globální úhel = 180°
    otevri_klepeta();
    otevri_prepazku();

    ///////////////////////////////////// sesta (180°)
    draha = 470 - vzdalenost_od_zdi;
    jed_a_sbirej_kostky_mm_gyro(draha, sbirej, 180.0f);
    otevri_vsechny_zasobniky();
    forward_gyro(240, 15, 180.0f);
    delay(500);
    zavri_vsechny_zasobniky();
    rkLedGreen(false);
    rkLedRed(false);
}


void setup() {
    Serial.begin(115200);
    rkConfig cfg;
    rkSetup(cfg);
    left_id = rb::MotorId(cfg.motor_id_left - 1);
    right_id = rb::MotorId(cfg.motor_id_right - 1);
    wheel_circumference = M_PI * cfg.motor_wheel_diameter;
    max_speed = cfg.motor_max_ticks_per_second;
    prevod_motoru = cfg.prevod_motoru;
    wheel_circumference_left= cfg.left_wheel_diameter * M_PI;
    wheel_circumference_right= cfg.right_wheel_diameter * M_PI;
    polarity_switch_left = cfg.motor_polarity_switch_left;
    polarity_switch_right = cfg.motor_polarity_switch_right;
    Button1 = cfg.Button1;
    Button2 = cfg.Button2;


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rkLedBlue(true);

    // Inicializace a kalibrace vestavěného gyroskopu (robot musí být při startu v klidu)
    gyroInit();

    init_ruka();

    pinMode(21, PULLUP);
    pinMode(22, PULLUP);

    Wire1.begin(21, 22, 400000);;
    Wire1.setTimeOut(1);

    delay(500);

    rkColorSensorInit("klepeta_senzor", Wire1, tcs1);

    // ruka_nahoru();
    // delay(5000);
    // ruka_dolu();

    otevri_klepeta();

    chytaniSemafor = xSemaphoreCreateBinary();
    xSemaphoreGive(chytaniSemafor); 
}


void loop() {
    // Pro zjištění správné osy gyroskopu (X / Y / Z) a směru odkomentuj řádek níže a otáčej robotem na stole:
    gyroDebugPrintAxes();

    if (rkButtonIsPressed(BTN_UP)) {
        rkLedGreen(true);
        Serial.println("=== TEST GYRO: START (0°) ===");
        gyroReset();
        delay(300);

        Serial.printf("[TEST] Pred otocenim doleva: %.2f°\n", gyroGetAngle360());
        turn_gyro(270.0f, 20); // 90° doleva (0° -> 270°)
        Serial.printf("[TEST] Po otoceni doleva: %.2f°\n", gyroGetAngle360());
        delay(300);

        Serial.println("[TEST] Jedu 1000 mm rovne podle gyra (cilovy uhel 270°)...");
        forward_gyro(1000, 30, 270.0f);
        Serial.printf("[TEST] Po dojezdu 1000 mm: %.2f°\n", gyroGetAngle360());
        delay(300);

        Serial.println("[TEST] Otacim se o 90° doprava (270° -> 0°)...");
        turn_gyro(0.0f, 20);   // 90° doprava (270° -> 0°)
        Serial.printf("[TEST] Po otoceni doprava (KONEC): %.2f°\n", gyroGetAngle360());

        rkLedGreen(false);
    }
    if (rkButtonIsPressed(BTN_ON)) {
        vyhrej_to_gyro();
    }
    if (rkButtonIsPressed(BTN_RIGHT)) {
        nachystej_se();
    }
    if (rkButtonIsPressed(BTN_LEFT)) {
        rkLedYellow(true);
        // Testovací úsek: začínáme směrem doleva (globální úhel 270° -> po otočení 180° -> 90°)
        global_heading = 270.0f;
        gyroSetAngle(270.0f);

        jed_a_sbirej_kostky_buttons_gyro(270.0f);
        backward_gyro(dozadu_kratce, 30, 270.0f);
        int vzdalenost_od_zdi = rkUltraMeasure(1);
        delay(50);
        turn_gyro(180.0f, 20); // otočení doleva o 90° z 270° na 180°
        delay(100);
        otevri_klepeta();
        otevri_prepazku();
        /////////////////////////////////////pata
        int draha = 880 - vzdalenost_od_zdi;// 720 + 160 
        jed_a_sbirej_kostky_mm_gyro(draha, true, 180.0f);

        delay(100);
        vzdalenost_od_zdi = rkUltraMeasure(1);
        delay(50);
        turn_gyro(90.0f, 20); // otočení doleva o 90° ze 180° na 90°
        otevri_klepeta();
        otevri_prepazku();
        /////////////////////////////////////sesta
        draha = 470 - vzdalenost_od_zdi;// 720 + 160
        jed_a_sbirej_kostky_mm_gyro(draha, true, 90.0f);
        otevri_vsechny_zasobniky();
        forward_gyro(220, 15, 90.0f);
        delay(500);
        zavri_vsechny_zasobniky();
        rkLedYellow(false);
    }
    delay(40);
}