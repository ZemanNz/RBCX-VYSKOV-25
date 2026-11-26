#pragma once
#include "robotka.h"
#include "robot_pohyb.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// ============================================================================
// ZÁLOHA PŮVODNÍCH POHYBOVÝCH FUNKCÍ BEZ GYROSKOPU (pouze enkodéry + ultrazvuk)
// Tento soubor lze kdykoliv smazat, jakmile potvrdíte gyro.
// ============================================================================

extern int dozadu_dlouze;
extern int dozadu_kratce;

// Dopředné deklarace funkcí z robot_commands.h (aby IDE nehlásilo chyby)
void otevri_prepazku();
void zavri_prepazku();
bool je_tam_kostka_ir();
void chyt_a_uloz_kostku();
void otevri_klepeta();
void otevri_vsechny_zasobniky();
void zavri_vsechny_zasobniky();

// ============================================================================
// PŮVODNÍ SBÍRACÍ FUNKCE (POUZE ENKODÉRY - ZACHOVÁNY BEZE ZMĚNY)
// ============================================================================

inline void jed_a_sbirej_kostky_mm(float mm, bool sbirej) {
    auto& man = rb::Manager::get();
    int speed = 30;
    if (sbirej == false) {
        speed = 50;
    }

    float m_kp = 0.23f;
    float m_min_speed = 20.0f;
    float m_max_correction = 8.5f;
    man.motor(left_id).setCurrentPosition(0);
    man.motor(right_id).setCurrentPosition(0);

    int target_ticks_left = mmToTicks(mm);
    int target_ticks_right = mmToTicks(mm);
    float left_pos = 0;
    float right_pos = 0;
    float progres_left = 0.0f;
    float progres_right = 0.0f;
    float rozdil_progres = 0.0f;
    bool left_done = false;
    bool right_done = false;

    float base_speed_left = polarity_switch_left ? -speed : speed;
    float base_speed_right = polarity_switch_right ? -speed : speed;

    unsigned long start_time = millis();
    int timeoutMs = 30000 * (mm / 400);

    otevri_prepazku();
    delay(500);

    while ((millis() - start_time < timeoutMs) && (target_ticks_left > abs(left_pos) || target_ticks_right > abs(right_pos))) {
        man.motor(left_id).requestInfo([&](rb::Motor& info) {
            left_pos = info.position();
        });
        man.motor(right_id).requestInfo([&](rb::Motor& info) {
            right_pos = info.position();
        });

        delay(10);

        progres_left = (float(abs(left_pos)) / float(target_ticks_left));
        progres_right = (float(abs(right_pos)) / float(target_ticks_right));
        rozdil_progres = progres_left - progres_right;

        float correction = rozdil_progres * m_kp * 1800;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));

        float speed_left = base_speed_left;
        float speed_right = base_speed_right;

        if (correction > 0) {
            if (polarity_switch_left) {
                speed_left += correction;
                speed_right += correction;
            } else {
                speed_left -= correction;
                speed_right -= correction;
            }
        } else if (correction < 0) {
            if (polarity_switch_right) {
                speed_right -= correction;
                speed_left -= correction;
            } else {
                speed_right += correction;
                speed_left += correction;
            }
        }

        if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
            speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
        }
        if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
            speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
        }

        man.motor(left_id).speed(pctToSpeed(speed_left));
        man.motor(right_id).speed(pctToSpeed(speed_right));

        std::cout << "Button1: " << digitalRead(Button1) << " Button2: " << digitalRead(Button2) << std::endl;

        if ((digitalRead(Button1) == LOW) && !left_done) {
            start_time = millis();
            timeoutMs = 3000;
            left_done = true;
        }
        if ((digitalRead(Button2) == LOW) && !right_done) {
            start_time = millis();
            timeoutMs = 3000;
            right_done = true;
        }
        if (left_done && right_done) {
            delay(50);
            break;
        }

        if (je_tam_kostka_ir() && sbirej) {
            delay(400);
            man.motor(left_id).speed(0);
            man.motor(right_id).speed(0);
            chyt_a_uloz_kostku();
        }
    }

    man.motor(left_id).speed(0);
    man.motor(right_id).speed(0);
    man.motor(left_id).power(0);
    man.motor(right_id).power(0);
    if (je_tam_kostka_ir() && sbirej) {
        chyt_a_uloz_kostku();
    }
    zavri_prepazku();
    delay(500);
}

inline void jed_a_sbirej_kostky_buttons() {
    auto& man = rb::Manager::get();
    int speed = 30;
    int mm = 7 * 700;
    int posbyrane_kostky = 0;

    float m_kp = 0.23f;
    float m_min_speed = 20.0f;
    float m_max_correction = 8.5f;
    man.motor(left_id).setCurrentPosition(0);
    man.motor(right_id).setCurrentPosition(0);

    int target_ticks_left = mmToTicks(mm);
    int target_ticks_right = mmToTicks(mm);
    float left_pos = 0;
    float right_pos = 0;
    float progres_left = 0.0f;
    float progres_right = 0.0f;
    float rozdil_progres = 0.0f;
    bool left_done = false;
    bool right_done = false;

    float base_speed_left = polarity_switch_left ? -speed : speed;
    float base_speed_right = polarity_switch_right ? -speed : speed;

    unsigned long start_time = millis();
    int timeoutMs = 30000 * (mm / 400);

    otevri_prepazku();
    delay(500);

    while ((millis() - start_time < timeoutMs)) {
        man.motor(left_id).requestInfo([&](rb::Motor& info) {
            left_pos = info.position();
        });
        man.motor(right_id).requestInfo([&](rb::Motor& info) {
            right_pos = info.position();
        });

        delay(10);

        progres_left = (float(abs(left_pos)) / float(target_ticks_left));
        progres_right = (float(abs(right_pos)) / float(target_ticks_right));
        rozdil_progres = progres_left - progres_right;

        float correction = rozdil_progres * m_kp * 1800;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));

        float speed_left = base_speed_left;
        float speed_right = base_speed_right;

        if (correction > 0) {
            if (polarity_switch_left) {
                speed_left += correction;
                speed_right += correction;
            } else {
                speed_left -= correction;
                speed_right -= correction;
            }
        } else if (correction < 0) {
            if (polarity_switch_right) {
                speed_right -= correction;
                speed_left -= correction;
            } else {
                speed_right += correction;
                speed_left += correction;
            }
        }

        if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
            speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
        }
        if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
            speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
        }

        man.motor(left_id).speed(pctToSpeed(speed_left));
        man.motor(right_id).speed(pctToSpeed(speed_right));

        std::cout << "Button1: " << digitalRead(Button1) << " Button2: " << digitalRead(Button2) << std::endl;

        if ((digitalRead(Button1) == LOW) && !left_done) {
            start_time = millis();
            timeoutMs = 3000;
            left_done = true;
        }
        if ((digitalRead(Button2) == LOW) && !right_done) {
            start_time = millis();
            timeoutMs = 3000;
            right_done = true;
        }
        if (left_done && right_done) {
            delay(50);
            break;
        }
        if (je_tam_kostka_ir()) {
            delay(400);
            man.motor(left_id).speed(0);
            man.motor(right_id).speed(0);
            posbyrane_kostky++;
            chyt_a_uloz_kostku();
        }
    }

    man.motor(left_id).speed(0);
    man.motor(right_id).speed(0);
    man.motor(left_id).power(0);
    man.motor(right_id).power(0);
    if (je_tam_kostka_ir()) {
        posbyrane_kostky++;
        chyt_a_uloz_kostku();
    }
    zavri_prepazku();
    delay(500);
}

inline void jed_a_sbirej_kostky_buttons_fast() {
    auto& man = rb::Manager::get();
    int speed = 30;
    int mm = 7 * 700;
    int posbyrane_kostky = 0;

    float m_kp = 0.23f;
    float m_min_speed = 20.0f;
    float m_max_correction = 8.5f;
    man.motor(left_id).setCurrentPosition(0);
    man.motor(right_id).setCurrentPosition(0);

    int target_ticks_left = mmToTicks(mm);
    int target_ticks_right = mmToTicks(mm);
    float left_pos = 0;
    float right_pos = 0;
    float progres_left = 0.0f;
    float progres_right = 0.0f;
    float rozdil_progres = 0.0f;
    bool left_done = false;
    bool right_done = false;

    float base_speed_left = polarity_switch_left ? -speed : speed;
    float base_speed_right = polarity_switch_right ? -speed : speed;

    unsigned long start_time = millis();
    int timeoutMs = 30000 * (mm / 400);

    otevri_prepazku();
    delay(500);

    while ((millis() - start_time < timeoutMs)) {
        man.motor(left_id).requestInfo([&](rb::Motor& info) {
            left_pos = info.position();
        });
        man.motor(right_id).requestInfo([&](rb::Motor& info) {
            right_pos = info.position();
        });

        delay(10);

        progres_left = (float(abs(left_pos)) / float(target_ticks_left));
        progres_right = (float(abs(right_pos)) / float(target_ticks_right));
        rozdil_progres = progres_left - progres_right;

        float correction = rozdil_progres * m_kp * 1800;
        correction = std::max(-m_max_correction, std::min(correction, m_max_correction));

        float speed_left = base_speed_left;
        float speed_right = base_speed_right;

        if (correction > 0) {
            if (polarity_switch_left) {
                speed_left += correction;
                speed_right += correction;
            } else {
                speed_left -= correction;
                speed_right -= correction;
            }
        } else if (correction < 0) {
            if (polarity_switch_right) {
                speed_right -= correction;
                speed_left -= correction;
            } else {
                speed_right += correction;
                speed_left += correction;
            }
        }

        if (abs(speed_left) < m_min_speed && abs(speed_left) > 0) {
            speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
        }
        if (abs(speed_right) < m_min_speed && abs(speed_right) > 0) {
            speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
        }

        man.motor(left_id).speed(pctToSpeed(speed_left));
        man.motor(right_id).speed(pctToSpeed(speed_right));

        if ((digitalRead(Button1) == LOW) && !left_done) {
            start_time = millis();
            timeoutMs = 3000;
            left_done = true;
        }
        if ((digitalRead(Button2) == LOW) && !right_done) {
            start_time = millis();
            timeoutMs = 3000;
            right_done = true;
        }
        if (left_done && right_done) {
            delay(50);
            break;
        }

        if (je_tam_kostka_ir()) {
            posbyrane_kostky++;
            if (xSemaphoreTake(chytaniSemafor, 0) == pdTRUE) {
                BaseType_t res = xTaskCreatePinnedToCore(
                    taskChytAUlozKostku,
                    "ChytKostku",
                    4096,
                    NULL,
                    1,
                    &taskChytHandle,
                    0
                );
                if (res != pdPASS) {
                    taskChytHandle = NULL;
                    xSemaphoreGive(chytaniSemafor);
                    std::cout << "ERROR: task create failed" << std::endl;
                }
            }
        }
    }

    man.motor(left_id).speed(0);
    man.motor(right_id).speed(0);
    man.motor(left_id).power(0);
    man.motor(right_id).power(0);

    zavri_prepazku();
    delay(500);
}


void vyhrej_to_rychlejsi(){// rychlejsi v rohach asinchroni sbirani hostek
    rkLedGreen(true);
    bool sbirej = true;
    int start_time = millis();
    /////////////////////////////////////prvni 
    
    //forward(20, 20);
    //radius_left(70, 90, 40);
    jed_a_sbirej_kostky_buttons_fast();
    
    backward(dozadu_kratce, 30);        
    turn_on_spot_left(90, 40);
    delay(1000);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    jed_a_sbirej_kostky_buttons();
    backward(dozadu_dlouze, 30);
    turn_on_spot_left(90, 40);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////druha
    jed_a_sbirej_kostky_buttons();
    backward(dozadu_kratce, 30);
    turn_on_spot_left(90, 40);
    delay(100);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////treti
    jed_a_sbirej_kostky_buttons_fast();
    backward(dozadu_dlouze, 30);
    int vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);

    if(vzdalenost_od_zdi < 90) { // do toho vzdalenyho rohu
        turn_on_spot_right(30, 40);
        backward(dozadu_dlouze, 30);
        turn_on_spot_left(30, 40);
        delay(100);
        jed_a_sbirej_kostky_buttons_fast();
        backward(150, 30);
    }
        
    turn_on_spot_left(90, 40);
    delay(100);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////ctvrta
    jed_a_sbirej_kostky_buttons_fast();
    backward(dozadu_kratce, 30);
    vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);
    turn_on_spot_left(90, 40);
    delay(100);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////pata

    int time_od_startu = millis() - start_time;
    std::cout<< "Cas od startu: " << time_od_startu << " ms" << std::endl;

    if(time_od_startu > 150000){ // 2.5 minuty --- dojet to stihne za 20 sekund
        std::cout<< "Musim rychle dojet" << std::endl;
        sbirej = false;
        rkLedRed(true);
    }
      
    int draha = 880 - vzdalenost_od_zdi;// 720 + 160
    jed_a_sbirej_kostky_mm(draha ,sbirej);

    delay(100);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);
    turn_on_spot_left(90, 40);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////sesta
    draha = 470 - vzdalenost_od_zdi;// 720 + 160
    jed_a_sbirej_kostky_mm(draha, sbirej);
    otevri_vsechny_zasobniky();
    forward_acc(240, 15);
    delay(500);
    zavri_vsechny_zasobniky();
    rkLedGreen(false);
    rkLedRed(false);
}

void vyhrej_to(){
    rkLedGreen(true);
    bool sbirej = true;
    int start_time = millis();
    /////////////////////////////////////prvni
    delay(1000);
    //forward(20, 20);
    //radius_left(70, 90, 40);
    jed_a_sbirej_kostky_buttons();
    backward(dozadu_kratce, 30);        
    turn_on_spot_left(90, 40);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    jed_a_sbirej_kostky_buttons();
    backward(dozadu_dlouze, 30);
    turn_on_spot_left(90, 40);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////druha
    jed_a_sbirej_kostky_buttons();
    backward(dozadu_kratce, 30);
    turn_on_spot_left(90, 40);
    delay(100);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////treti
    jed_a_sbirej_kostky_buttons();
    backward(dozadu_dlouze, 30);
    int vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);

    if(vzdalenost_od_zdi < 90) { // do toho vzdalenyho rohu
        turn_on_spot_right(30, 40);
        backward(dozadu_dlouze, 30);
        turn_on_spot_left(30, 40);
        delay(100);
        jed_a_sbirej_kostky_buttons();
        backward(150, 30);
    }
        
    turn_on_spot_left(90, 40);
    delay(100);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////ctvrta
    jed_a_sbirej_kostky_buttons();
    backward(dozadu_kratce, 30);
    vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);
    turn_on_spot_left(90, 40);
    delay(100);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////pata

    int time_od_startu = millis() - start_time;
    std::cout<< "Cas od startu: " << time_od_startu << " ms" << std::endl;

    if(time_od_startu > 150000){ // 2.5 minuty --- dojet to stihne za 20 sekund
        std::cout<< "Musim rychle dojet" << std::endl;
        sbirej = false;
        rkLedRed(true);
    }
      
    int draha = 880 - vzdalenost_od_zdi;// 720 + 160
    jed_a_sbirej_kostky_mm(draha ,sbirej);

    delay(100);
    orient_to_wall(true, []() -> uint32_t { return rkUltraMeasure(1); },
                         []() -> uint32_t { return rkUltraMeasure(2); }, 28);
    delay(100);
    vzdalenost_od_zdi = rkUltraMeasure(1);
    delay(50);
    turn_on_spot_left(88, 40);
    otevri_klepeta();
    otevri_prepazku();
    /////////////////////////////////////sesta
    draha = 470 - vzdalenost_od_zdi;// 720 + 160
    jed_a_sbirej_kostky_mm(draha, sbirej);
    otevri_vsechny_zasobniky();
    forward_acc(240, 15);
    delay(500);
    zavri_vsechny_zasobniky();
    rkLedGreen(false);
    rkLedRed(false);

}
