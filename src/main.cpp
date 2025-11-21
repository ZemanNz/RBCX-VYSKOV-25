#include "robotka.h"
#include "robot_commands.h"

rb::MotorId left_id;
rb::MotorId right_id;
int max_speed;
float wheel_circumference;
float prevod_motoru;
float wheel_circumference_left;
float wheel_circumference_right;
bool polarity_switch_left;
bool polarity_switch_right;

Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
float r1, g1, b1;


int16_t pctToSpeed(float pct) {
    pct = rb::clamp(pct, -100.0f, 100.0f);
    int32_t speed = static_cast<int32_t>((pct * max_speed) / 100.0f);
    return rb::clamp(speed, -INT16_MAX, INT16_MAX);
}

int32_t mmToTicks(float mm){
    return (mm / wheel_circumference) * prevod_motoru;
}
int32_t mmToTicks_left(float mm){
    return (mm / wheel_circumference_left) * prevod_motoru;
}
int32_t mmToTicks_right(float mm){
    return (mm / wheel_circumference_right) * prevod_motoru;
}

void jed_a_sbirej_kostky(float mm) {
    auto& man = rb::Manager::get();
    int speed = 20;
    
    float m_kp = 0.23f; // Proporcionální konstanta
    float m_min_speed = 20.0f; // Minimální rychlost motorů
    float m_max_correction = 8.5f;
    // Reset pozic
    man.motor(left_id).setCurrentPosition(0);
    man.motor(right_id).setCurrentPosition(0);

    int target_ticks_left = mmToTicks(mm);
    int target_ticks_right = mmToTicks(mm);
    float left_pos = 0;
    float right_pos = 0;
    float progres_left = 0.0f;
    float progres_right = 0.0f;
    float rozdil_progres = 0.0f;
    // std::cout << "Target ticks left: " << target_ticks_left << " tick right" << target_ticks_right << std::endl;
    // Základní rychlosti s přihlédnutím k polaritě
    float base_speed_left = polarity_switch_left ? -speed : speed;
    float base_speed_right = polarity_switch_right ? -speed : speed;
    
    unsigned long start_time = millis();
    unsigned long start_time_for_try = millis();
    int timeoutMs =30000;
    int time_to_try = 2000;
    
    while((target_ticks_left > abs(left_pos) || target_ticks_right > abs(right_pos)) && 
          (millis() - start_time < timeoutMs)) {
        
        // Čtení pozic
        man.motor(left_id).requestInfo([&](rb::Motor& info) {
             left_pos = info.position();
          });
        man.motor(right_id).requestInfo([&](rb::Motor& info) {
             right_pos = info.position();
          });

        delay(10);

        // std::cout << "Left pos: " << left_pos << ", Right pos: " << right_pos << std::endl;
        //print_wifi("Left pos: " + String(left_pos) + " Right pos: " + String(right_pos) + "\n");
        // P regulátor - pracuje s absolutními hodnotami pozic
        progres_left = (float(abs(left_pos)) / float(target_ticks_left));
        progres_right = (float(abs(right_pos)) / float(target_ticks_right));
        rozdil_progres = progres_left - progres_right;


        float correction = rozdil_progres * m_kp * 1800;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        // std::cout << "Progres L: " << progres_left << ", Progres R: " << progres_right << ", Diff: " << rozdil_progres << ", Correction: " << correction << std::endl;
        // Výpočet korigovaných rychlostí
        float speed_left = base_speed_left;
        float speed_right = base_speed_right;
        
        // Aplikace korekce podle polarity
        if (correction > 0) {
            // Levý je napřed - zpomalit levý
            if (polarity_switch_left) {
                speed_left += correction;  // Přidat k záporné rychlosti = zpomalit
                speed_right += correction;  // Odečíst od kladné rychlosti = zrychlit
            } else {
                speed_left -= correction;  // Odečíst od kladné rychlosti = zpomalit
                speed_right -= correction;  // Přidat ke kladné rychlosti = zrychlit
            }
        } else if (correction < 0) {
            // Pravý je napřed - zpomalit pravý
            if (polarity_switch_right) {
                speed_right -= correction;  // Odečíst od záporné rychlosti = zpomalit
                speed_left -= correction;  // Přidat k záporné rychlosti = zrychlit
            } else {
                speed_right += correction;  // Přidat ke kladné rychlosti = zpomalit
                speed_left += correction;  // Odečíst od kladné rychlosti = zrychlit
            }
        }
        
        // Zajištění minimální rychlosti
        if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
            speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
        }
        if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
            speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
        }
        
        // Nastavení výkonu motorů
        man.motor(left_id).speed(pctToSpeed(speed_left ));
        man.motor(right_id).speed(pctToSpeed(speed_right ));
        // std::cout << "Speed left: " << speed_left << ", Speed right: " << speed_right << std::endl;
        //print_wifi("Speed left: " + String(speed_left) + " Speed right: " + String(speed_right) + "\n");

        if(time_to_try < millis() - start_time_for_try){
            man.motor(left_id).speed(0);
            man.motor(right_id).speed(0);
            delay(100);
            start_time_for_try = millis();
            if(try_to_catch()){ // je tam kostka
                chyt_a_uloz_kostku();
            }
        }
    }
    
    // Zastavení motorů
    man.motor(left_id).speed(0);
    man.motor(right_id).speed(0);
    man.motor(left_id).power(0);
    man.motor(right_id).power(0);
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


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    rkLedBlue(true);

    init_ruka();

    pinMode(21, PULLUP);
    pinMode(22, PULLUP);

    Wire1.begin(21, 22, 400000);
    Wire1.setTimeOut(1);

    rkColorSensorInit("klepeta_senzor", Wire1, tcs1);

    //zavri_vsechny_zasobniky();

}

void loop() {

    if (rkButtonIsPressed(BTN_UP)) {
        //otoc_motorem(240, true);
        //otevri_vsechny_zasobniky();
        natocit_ruku(1); // 1 je na nabirani a G
    }
    if (rkButtonIsPressed(BTN_RIGHT)) {
        forward(1000, 80);
    }
    if (rkButtonIsPressed(BTN_LEFT)) {
        //ruka_dolu();
        //natocit_ruku(2);
        chyt_a_uloz_kostku();
    }
}