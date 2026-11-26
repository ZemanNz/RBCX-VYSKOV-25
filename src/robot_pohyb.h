#pragma once

#include "robotka.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// ============================================================================
// EXTERNÍ PROMĚNNÉ Z main.cpp (nastavené v setup() z rkConfig)
// ============================================================================
extern rb::MotorId left_id;
extern rb::MotorId right_id;
extern int max_speed;
extern float wheel_circumference;
extern float prevod_motoru;
extern float wheel_circumference_left;
extern float wheel_circumference_right;
extern bool polarity_switch_left;
extern bool polarity_switch_right;
extern byte Button1;
extern byte Button2;

extern TaskHandle_t taskChytHandle;
extern SemaphoreHandle_t chytaniSemafor;
extern void taskChytAUlozKostku(void *pvParameters);

// Dopředné deklarace funkcí z robot_commands.h (aby IDE nehlásilo chyby)
void otevri_prepazku();
void zavri_prepazku();
bool je_tam_kostka_ir();
void chyt_a_uloz_kostku();

// ============================================================================
// PŘEHLED POHYBOVÝCH FUNKCÍ (POROVNÁNÍ PŮVODNÍCH A GYRO VERZÍ)
// ============================================================================
// 1) Původní funkce z knihovny robotka.h (pouze enkodéry, bez gyroskopu):
//    - forward(), backward(), forward_acc(), backward_acc()
//    - turn_on_spot_left(), turn_on_spot_right()
//    - orient_to_wall()  <-- dorovnávání ultrazvuky (u gyro tras VYPNUTO)
//
// 2) Původní sbírací funkce (pouze enkodéry, zachovány pro porovnání/zálohu):
//    - jed_a_sbirej_kostky_mm()
//    - jed_a_sbirej_kostky_buttons()
//    - jed_a_sbirej_kostky_buttons_fast()
//    Nevýhoda: při prokluzu kol nebo nárazu do kostky robot uhne ze směru.
//
// 3) Nové gyroskopické funkce (podle projektu Anténka, na vestavěném MPU6050):
//    - turn_gyro(target_angle, speed) + align_gyro()
//        Oproti turn_on_spot_* nepoužívá odhad z enkodérů, ale měří skutečné
//        natočení gyroskopem. Má rozjezdovou/brzdnou rampu, předstih zastavení,
//        Anti-Stall (přidá výkon, pokud se robot zastaví kvůli tření) a jemné
//        dotažení align_gyro pod 0.8°.
//    - turn_gyro_left() / turn_gyro_right()
//        Posunou globální úhel v kruhu [0°, 360°) a otočí na něj.
//    - move_straight_gyro() / forward_gyro() / backward_gyro()
//        Oproti forward/backward drží během jízdy přímý směr podle gyroskopu
//        (P-regulátor vůči target_heading), má S-křivkový rozjezd, brzdění
//        a pomalý crawl dojezd.
//    - jed_a_sbirej_kostky_mm_gyro()
//    - jed_a_sbirej_kostky_buttons_gyro()
//    - jed_a_sbirej_kostky_buttons_fast_gyro()
//        Alternativy původních sbíracích funkcí: drží směr podle globálního
//        úhlu a při sepnutí OBĚMA předními tlačítky o stěnu automaticky
//        vynulují drift gyroskopu přesně na úhel dané stěny (gyroSetAngle).
// ============================================================================


// ============================================================================
// KONFIGURACE VESTAVĚNÉHO GYROSKOPU (TADY SE MĚNÍ OSA A SMĚR!)
// ============================================================================

enum class GyroAxis {
    X,
    Y,
    Z
};

// 1. VÝBĚR OSY GYROSKOPU:
//    Podle toho, jak je deska RBCX v robotovi natočená (svisle / naležato),
//    vyber osu, kolem které se robot otáčí na podlaze:
//      GyroAxis::X  |  GyroAxis::Y  |  GyroAxis::Z
//    (Pro zjištění správné osy zavolej gyroDebugPrintAxes() a otoč robotem na stole.)
const GyroAxis GYRO_AXIS = GyroAxis::X; // <-- PŘEPNI NA GyroAxis::X, GyroAxis::Y nebo GyroAxis::Z

// 2. SMĚR OSY (POLARITA):
//    Chceme kompasový globální úhel:
//      - Otočení DOLEVA:  0° -> 270° (-90°) -> 180° -> 90° -> 0° (úhel KLESÁ)
//      - Otočení DOPRAVA: 0° -> 90° -> 180° -> 270° -> 0°        (úhel ROSTE)
//    Pokud při otočení robota rukou DOLEVA úhel místo klesání roste, změň -1.0f na 1.0f (nebo naopak).
const float GYRO_DIRECTION = -1.0f;

// 3. KALIBRACE MĚŘÍTKA:
//    Pokud se při příkazu 90° robot otočí reálně např. o 88° nebo 92°,
//    uprav tuto konstantu (např. 1.02f nebo 0.98f).
const float GYRO_SCALE_CORRECTION = 1.00f;

// Globální cílový směr robota v hřišti [0°, 360°)
float global_heading = 0.0f;

struct GyroState {
    float angle = 0.0f;
    float offset = 0.0f;
    float rawOffset = 0.0f;
    bool initialized = false;
    volatile bool calibrateRequested = false;
    volatile bool isCalibrating = false;

    float lastRawGyro = 0.0f;
    float lastFilteredGyro = 0.0f;
};

inline GyroState& getGyroState() {
    static GyroState state;
    return state;
}

// Přečte aktuální úhlovou rychlost [deg/s] ze zvolené osy GYRO_AXIS
inline float gyroReadRawRateDegPerSec() {
    auto &man = rb::Manager::get();
    switch (GYRO_AXIS) {
        case GyroAxis::X: return man.mpu().getGyroX();
        case GyroAxis::Y: return man.mpu().getGyroY();
        case GyroAxis::Z:
        default:          return man.mpu().getGyroZ();
    }
}

// Normalizuje libovolný úhel do rozsahu [0°, 360°)
inline float normalizeAngle360(float angle) {
    float a = std::fmod(angle, 360.0f);
    if (a < 0.0f) a += 360.0f;
    if (a >= 360.0f) a -= 360.0f;
    return a;
}

// Nejkratší rozdíl úhlů (target - current) v rozsahu (-180°, +180°]
// Záporná hodnota (< 0) = cíl je VLEVO, kladná hodnota (> 0) = cíl je VPRAVO
inline float gyroAngleDiff(float target_deg, float current_deg) {
    float diff = std::fmod(target_deg - current_deg, 360.0f);
    if (diff > 180.0f) diff -= 360.0f;
    if (diff <= -180.0f) diff += 360.0f;
    return diff;
}

inline void gyroTask(void *pvParameters) {
    auto &state = getGyroState();
    unsigned long lastTime = micros();

    while (true) {
        if (state.calibrateRequested) {
            state.calibrateRequested = false;
            state.isCalibrating = true;
            float sum = 0.0f;
            const int samples = 100;
            for (int i = 0; i < samples; i++) {
                sum += gyroReadRawRateDegPerSec();
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            state.rawOffset = sum / (float)samples;
            state.angle = 0.0f;
            state.offset = 0.0f;
            state.lastFilteredGyro = 0.0f;
            lastTime = micros();
            state.isCalibrating = false;
            Serial.printf("[gyroTask] Recalibrated. new rawOffset: %.4f deg/s\n", state.rawOffset);
            continue;
        }

        unsigned long currentTime = micros();
        float dt = (float)(currentTime - lastTime) / 1000000.0f;
        if (dt > 0.05f) dt = 0.05f;
        lastTime = currentTime;

        float g_deg_s = gyroReadRawRateDegPerSec();
        state.lastRawGyro = g_deg_s;
        float raw_g = g_deg_s - state.rawOffset;

        // Široký filtr pro odhození pouze extrémního šumu (>1700 deg/s ~= 30 rad/s)
        float g_val = 0.0f;
        if (std::abs(raw_g) < 1700.0f) {
            g_val = raw_g;
            state.lastFilteredGyro = raw_g;
        } else {
            g_val = state.lastFilteredGyro;
        }

        // Mrtvé pásmo (deadband) pro eliminaci driftu v naprostém klidu (~0.5 deg/s)
        if (std::abs(g_val) < 0.5f) g_val = 0.0f;

        // Integrace úhlové rychlosti do globálního úhlu
        state.angle += g_val * dt * GYRO_SCALE_CORRECTION * GYRO_DIRECTION;

        vTaskDelay(pdMS_TO_TICKS(1)); // 1000 Hz smyčka jako v Anténce
    }
}

/**
 * \brief Inicializuje vestavěný MPU6050 přes STM32, vypne kompresi (setCompressCoef(1)),
 *        zkalibruje offset ze 100 vzorků v klidu a spustí 1000 Hz gyroTask.
 */
inline void gyroInit() {
    auto &state = getGyroState();
    if (state.initialized) {
        Serial.println("[gyroInit] Gyro is already initialized.");
        return;
    }

    auto &man = rb::Manager::get();
    Serial.println("[gyroInit] Inicializace vestavěného MPU6050 přes STM32...");
    man.mpu().init();
    man.mpu().sendStart();
    // Kompresní koeficient 1 = STM32 posílá každé měření okamžitě bez zpoždění
    man.mpu().setCompressCoef(1);
    man.mpu().clearCalibrationData();

    Serial.println("[gyroInit] Čekám 600 ms na ustálení dat z STM32...");
    delay(600);

    Serial.println("=============================================");
    Serial.println("Calibrating built-in MPU-6050... KEEP ROBOT STILL!");
    Serial.println("=============================================");

    float sum = 0.0f;
    const int samples = 100;
    for (int i = 0; i < samples; i++) {
        float g = gyroReadRawRateDegPerSec();
        sum += g;
        if (i < 5) {
            Serial.printf("Sample %d: X=%.2f, Y=%.2f, Z=%.2f deg/s (zvolena osa: %.2f)\n",
                          i, man.mpu().getGyroX(), man.mpu().getGyroY(), man.mpu().getGyroZ(), g);
        }
        delay(10);
    }

    state.rawOffset = sum / (float)samples;
    state.angle = 0.0f;
    state.offset = 0.0f;
    state.lastFilteredGyro = 0.0f;
    state.initialized = true;
    global_heading = 0.0f;

    Serial.printf("MPU-6050 Calibration completed. Calculated rawOffset: %.4f deg/s\n", state.rawOffset);
    Serial.println("=============================================");

    xTaskCreatePinnedToCore(
        gyroTask,
        "gyroTask",
        4096,
        NULL,
        2,
        NULL,
        1
    );
}

inline float gyroGetAngle() {
    auto &state = getGyroState();
    return state.angle - state.offset;
}

inline float gyroGetAngle360() {
    return normalizeAngle360(gyroGetAngle());
}

/**
 * \brief Zarovná (vynuluje drift) aktuálního úhlu gyroskopu na známý globální úhel stěny
 *        (volá se při sepnutí obou předních tlačítek o stěnu).
 */
inline void gyroSetAngle(float known_global_angle) {
    auto &state = getGyroState();
    state.offset = state.angle - known_global_angle;
    Serial.printf("[GYRO ALIGN WALL] Gyro srovnáno o tlačítka na globální úhel: %.1f° (360: %.1f°)\n",
                  known_global_angle, normalizeAngle360(known_global_angle));
}

inline void gyroReset() {
    global_heading = 0.0f;
    gyroSetAngle(0.0f);
}

/**
 * \brief Pomocná diagnostika pro zjištění správné osy (X, Y nebo Z) a směru.
 *        Zavolej v loop() při testování na stole a otáčej robotem rukou DOLEVA.
 *        Osa, jejíž hodnota výrazně vyskočí, patří do konstanty GYRO_AXIS.
 */
inline void gyroDebugPrintAxes() {
    auto &man = rb::Manager::get();
    Serial.printf("[GYRO AXES] raw X: %7.2f | Y: %7.2f | Z: %7.2f deg/s  ||  Angle360: %6.2f°\n",
                  man.mpu().getGyroX(), man.mpu().getGyroY(), man.mpu().getGyroZ(), gyroGetAngle360());
}

// ============================================================================
// POMOCNÉ PŘEPOČTY MOTORŮ A ENKODÉRŮ
// ============================================================================

inline int16_t pctToSpeed(float pct) {
    pct = rb::clamp(pct, -100.0f, 100.0f);
    int32_t speed = static_cast<int32_t>((pct * max_speed) / 100.0f);
    return rb::clamp(speed, -INT16_MAX, INT16_MAX);
}

inline int32_t mmToTicks(float mm) {
    return (mm / wheel_circumference) * prevod_motoru;
}

inline int32_t mmToTicks_left(float mm) {
    return (mm / wheel_circumference_left) * prevod_motoru;
}

inline int32_t mmToTicks_right(float mm) {
    return (mm / wheel_circumference_right) * prevod_motoru;
}

inline int8_t clampSpeedPct(float s) {
    if (s > 100.0f) return 100;
    if (s < -100.0f) return -100;
    return static_cast<int8_t>(std::round(s));
}

// ============================================================================
// NOVÉ POHYBOVÉ FUNKCE S GYROSKOPEM (PODLE ANTÉNKY)
// Globální úhel (kompasový):
//   Start = 0°
//   Otočení DOLEVA o 90°  -> 270° (neboli -90°) -> 180° -> 90° -> 0°
//   Otočení DOPRAVA o 90° -> 90° -> 180° -> 270° -> 0°
// ============================================================================

/**
 * \brief Jemné dorovnání na cílový globální úhel gyroskopu s anti-stallem a blikáním LED (podle Anténky).
 */
inline void align_gyro(float target_global_angle, float speed = 6.0f, uint32_t timeout_ms = 2000) {
    unsigned long start_time = millis();
    int loop_count = 0;
    bool led_state = false;

    float base_align_speed = std::max(4.0f, std::min(std::abs(speed), 15.0f));
    float last_check_angle = gyroGetAngle();
    int stall_counter = 0;
    float boost = 0.0f;

    while (millis() - start_time < timeout_ms) {
        float current_angle = gyroGetAngle();
        float error = gyroAngleDiff(target_global_angle, current_angle); // <0 = doleva, >0 = doprava
        float abs_error = std::abs(error);

        if (abs_error < 0.8f) {
            break;
        }

        loop_count++;
        if (loop_count % 10 == 0) {
            led_state = !led_state;
            rkLedGreen(led_state);

            if (std::abs(gyroAngleDiff(current_angle, last_check_angle)) < 0.08f) {
                stall_counter++;
                if (stall_counter >= 2) {
                    boost += 1.0f;
                    if (boost > 10.0f) boost = 10.0f;
                }
            } else {
                stall_counter = 0;
                boost = 0.0f;
            }
            last_check_angle = current_angle;
        }

        float active_speed = base_align_speed + boost;
        if (error < 0.0f) {
            // Cíl je VLEVO -> levé kolo dozadu (-), pravé dopředu (+)
            rkMotorsSetSpeed(clampSpeedPct(-active_speed), clampSpeedPct(active_speed));
        } else {
            // Cíl je VPRAVO -> levé kolo dopředu (+), pravé dozadu (-)
            rkMotorsSetSpeed(clampSpeedPct(active_speed), clampSpeedPct(-active_speed));
        }
        delay(10);
    }

    rkMotorsSetSpeed(0, 0);
    rkMotorsSetPower(0, 0);
    rkLedGreen(false);
}

/**
 * \brief Přesné otočení na místě na cílový globální úhel pomocí gyroskopu (podle Anténky).
 *        Oproti turn_on_spot_* využívá reálný úhel z MPU, brzdnou rampu, předstih,
 *        Anti-Stall kompenzaci a závěrečné dorovnání align_gyro().
 * \param target_global_angle Cílový globální úhel (např. 0, 270 / -90, 180, 90).
 */
inline void turn_gyro(float target_global_angle, float speed = 20.0f, uint32_t timeout_ms = 5000) {
    if (speed <= 0) return;

    global_heading = normalizeAngle360(target_global_angle);

    rkMotorsSetSpeed(0, 0);
    rkMotorsSetPower(0, 0);
    delay(150);

    float start_angle = gyroGetAngle();
    float delta_angle = gyroAngleDiff(target_global_angle, start_angle); // <0 = doleva, >0 = doprava

    if (std::abs(delta_angle) < 0.5f) return;

    // Limit rychlosti otáčení na místě pro maximální přesnost a proti saturaci ±250 deg/s
    float max_speed = std::abs(speed);
    if (max_speed > 22.0f) {
        max_speed = 22.0f;
    }

    // Parametry rychlostního profilu podle Anténky (přizpůsobené pro rozteč kol 255 mm)
    float min_speed = 8.0f;
    float ramp_up_deg = 15.0f;
    float ramp_down_deg = 30.0f;
    const float offset_deg = 2.0f; // Předstih vypnutí před cílem

    bool turn_left = (delta_angle < 0.0f);
    unsigned long start_time = millis();

    int loop_counter = 0;
    float last_check_angle = start_angle;
    int stall_counter = 0;
    float min_speed_boost = 0.0f;

    Serial.printf("[GyroTurn] Start: %.2f° -> Target: %.1f° (delta: %.2f°, smer: %s)\n",
                  gyroGetAngle360(), global_heading, delta_angle, turn_left ? "DOLEVA" : "DOPRAVA");

    while (true) {
        loop_counter++;
        if (millis() - start_time > timeout_ms) {
            Serial.println("[GyroTurn] WARN: Timeout!");
            break;
        }

        float current_angle = gyroGetAngle();
        float signed_rem = gyroAngleDiff(target_global_angle, current_angle);

        if (turn_left) {
            if (signed_rem >= -offset_deg) {
                break;
            }
        } else {
            if (signed_rem <= offset_deg) {
                break;
            }
        }

        float error = std::abs(signed_rem);
        float turned = std::abs(gyroAngleDiff(current_angle, start_angle));

        // Anti-stall kompenzace
        if (loop_counter % 10 == 0) {
            if (std::abs(gyroAngleDiff(current_angle, last_check_angle)) < 0.08f) {
                stall_counter++;
                if (stall_counter >= 2) {
                    min_speed_boost += 1.0f;
                    if (min_speed_boost > 12.0f) min_speed_boost = 12.0f;
                }
            } else {
                stall_counter = 0;
                min_speed_boost = 0.0f;
            }
            last_check_angle = current_angle;
        }

        float active_min_speed = min_speed + min_speed_boost;
        if (active_min_speed > max_speed) active_min_speed = max_speed;

        // Rozjezdová rampa
        float speed_accel = max_speed;
        if (turned < ramp_up_deg) {
            speed_accel = active_min_speed + (max_speed - active_min_speed) * (turned / ramp_up_deg);
        }

        // Brzdná rampa
        float speed_decel = max_speed;
        if (error < ramp_down_deg) {
            if (error >= 7.0f) {
                speed_decel = active_min_speed + (max_speed - active_min_speed) * ((error - 7.0f) / 23.0f);
            } else {
                speed_decel = active_min_speed;
            }
        }

        float curr_speed = std::min(speed_accel, speed_decel);

        if (loop_counter % 6 == 0) {
            Serial.printf("[GyroTurn] Angle: %.2f° | Target: %.1f° | Rem: %.2f° | Speed: %.1f%%\n",
                          gyroGetAngle360(), global_heading, signed_rem, curr_speed);
        }

        if (turn_left) {
            rkMotorsSetSpeed(clampSpeedPct(-curr_speed), clampSpeedPct(curr_speed));
        } else {
            rkMotorsSetSpeed(clampSpeedPct(curr_speed), clampSpeedPct(-curr_speed));
        }

        delay(5);
    }

    rkMotorsSetSpeed(0, 0);
    rkMotorsSetPower(0, 0);
    delay(120);

    // Pokud po doběhu zbývá odchylka >= 1.2°, jemně dotáhneme pomocí align_gyro
    if (std::abs(gyroAngleDiff(target_global_angle, gyroGetAngle())) >= 1.2f) {
        align_gyro(target_global_angle, 6.0f, 1200);
    }

    Serial.printf("[GyroTurn] Finished: Final angle: %.2f° (target: %.1f°, error: %.2f°)\n",
                  gyroGetAngle360(), global_heading, gyroAngleDiff(target_global_angle, gyroGetAngle()));
}

/**
 * \brief Otočení DOLEVA o delta_deg vůči aktuálnímu globálnímu úhlu (0° -> 270° -> 180° -> 90° -> 0°).
 */
inline void turn_gyro_left(float delta_deg = 90.0f, float speed = 20.0f) {
    global_heading = normalizeAngle360(global_heading - std::abs(delta_deg));
    turn_gyro(global_heading, speed);
}

/**
 * \brief Otočení DOPRAVA o delta_deg vůči aktuálnímu globálnímu úhlu (0° -> 90° -> 180° -> 270° -> 0°).
 */
inline void turn_gyro_right(float delta_deg = 90.0f, float speed = 20.0f) {
    global_heading = normalizeAngle360(global_heading + std::abs(delta_deg));
    turn_gyro(global_heading, speed);
}

/**
 * \brief Jízda rovně (vpřed nebo vzad) stabilizovaná gyroskopem k cílovému globálnímu úhlu (podle Anténky).
 *        Oproti forward()/backward() z robotka.h koriguje směr podle gyroskopu (ne jen podle enkodérů)
 *        a používá 4-fázový profil (S-křivka rozjezd -> plavba -> brzdění -> crawl dojezd).
 * \param mm Vzdálenost v mm (kladná = vpřed, záporná = vzad).
 * \param speed Maximální rychlost v % (0–100).
 * \param target_heading Cílový globální úhel (výchozí = aktuální global_heading).
 */
inline void move_straight_gyro(float mm, float speed, float target_heading = global_heading, uint32_t timeout_ms = 8000) {
    if (mm == 0 || speed == 0) return;

    auto &man = rb::Manager::get();
    bool reverse = (mm < 0.0f);
    float real_target_mm = std::abs(mm);
    int target_ticks = std::abs(mmToTicks(real_target_mm));

    man.motor(left_id).setCurrentPosition(0);
    man.motor(right_id).setCurrentPosition(0);
    delay(30);

    float abs_target_speed = std::abs(speed);
    if (abs_target_speed < 8.0f) abs_target_speed = 8.0f;
    if (abs_target_speed > 100.0f) abs_target_speed = 100.0f;

    float crawl_speed = 10.0f;
    float accel_time_ms = 500.0f;
    float decel_distance_mm = 80.0f;
    float crawl_distance_mm = 15.0f;

    if (real_target_mm < 120.0f) {
        decel_distance_mm = real_target_mm * 0.45f;
        crawl_distance_mm = real_target_mm * 0.15f;
        accel_time_ms = 250.0f;
    }

    float speed_sign = reverse ? -1.0f : 1.0f;
    float kp_gyro = 1.0f;

    unsigned long start_time = millis();
    float current_speed = crawl_speed;
    float left_pos = 0.0f;
    float right_pos = 0.0f;

    while (true) {
        unsigned long now = millis();
        float elapsed_ms = (float)(now - start_time);

        man.motor(left_id).requestInfo([&](rb::Motor& info) {
            left_pos = info.position();
        });
        man.motor(right_id).requestInfo([&](rb::Motor& info) {
            right_pos = info.position();
        });
        delay(10);

        float avg_ticks = (std::abs(left_pos) + std::abs(right_pos)) / 2.0f;
        float real_pos_mm = (avg_ticks / prevod_motoru) * wheel_circumference;

        if (avg_ticks >= target_ticks || real_pos_mm >= real_target_mm) {
            break;
        }

        if (elapsed_ms > (float)timeout_ms) {
            Serial.println("[move_straight_gyro] WARN: Timeout!");
            break;
        }

        float dist_remaining = real_target_mm - real_pos_mm;
        float desired_speed = abs_target_speed;

        if (dist_remaining <= crawl_distance_mm) {
            desired_speed = crawl_speed;
        } else if (dist_remaining <= decel_distance_mm) {
            float decel_progress = (dist_remaining - crawl_distance_mm) / (decel_distance_mm - crawl_distance_mm);
            desired_speed = crawl_speed + (abs_target_speed - crawl_speed) * decel_progress;
        } else if (elapsed_ms < accel_time_ms) {
            float accel_progress = elapsed_ms / accel_time_ms;
            float smooth = accel_progress * accel_progress * (3.0f - 2.0f * accel_progress);
            desired_speed = crawl_speed + (abs_target_speed - crawl_speed) * smooth;
        }

        // Anti-jerk filtr
        float max_speed_change = 3.0f;
        if (desired_speed > current_speed + max_speed_change) {
            current_speed += max_speed_change;
        } else if (desired_speed < current_speed - max_speed_change) {
            current_speed -= max_speed_change;
        } else {
            current_speed = desired_speed;
        }
        if (current_speed < crawl_speed) current_speed = crawl_speed;

        // Gyro korekce směru:
        // heading_error > 0 => cíl je VPRAVO (robot uhnul doleva) => levé kolo zrychlit (+), pravé zpomalit (-)
        float heading_error = gyroAngleDiff(target_heading, gyroGetAngle());
        float correction = heading_error * kp_gyro;
        correction = std::max(-10.0f, std::min(correction, 10.0f));
        if (reverse) {
            correction = -correction;
        }

        float speed_l = current_speed + correction;
        float speed_r = current_speed - correction;

        rkMotorsSetSpeed(clampSpeedPct(speed_l * speed_sign), clampSpeedPct(speed_r * speed_sign));
    }

    rkMotorsSetSpeed(0, 0);
    rkMotorsSetPower(0, 0);
}

inline void forward_gyro(float mm, float speed, float target_heading = global_heading) {
    move_straight_gyro(std::abs(mm), speed, target_heading);
}

inline void backward_gyro(float mm, float speed, float target_heading = global_heading) {
    move_straight_gyro(-std::abs(mm), speed, target_heading);
}

// ============================================================================
// ALTERNATIVNÍ FUNKCE SBÍRÁNÍ KOSTEK S GYROSKOPEM A NULOVÁNÍM O TLAČÍTKA
// ============================================================================

/**
 * \brief Alternativa k jed_a_sbirej_kostky_mm() doplněná o držení směru gyroskopem.
 *        Pokud během jízdy narazí oběma tlačítky na stěnu, srovná gyroskop na target_heading.
 */
inline void jed_a_sbirej_kostky_mm_gyro(float mm, bool sbirej, float target_heading = global_heading) {
    auto& man = rb::Manager::get();
    int speed = sbirej ? 30 : 50;

    float kp_gyro = 1.2f;
    float m_min_speed = 18.0f;
    float m_max_correction = 8.5f;

    man.motor(left_id).setCurrentPosition(0);
    man.motor(right_id).setCurrentPosition(0);

    int target_ticks_left = mmToTicks(mm);
    int target_ticks_right = mmToTicks(mm);
    float left_pos = 0;
    float right_pos = 0;
    bool left_done = false;
    bool right_done = false;

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

        float correction = 0.0f;
        if (!left_done && !right_done) {
            float heading_error = gyroAngleDiff(target_heading, gyroGetAngle());
            correction = heading_error * kp_gyro;
            correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        }

        float fwd_left = std::max(m_min_speed, (float)speed + correction);
        float fwd_right = std::max(m_min_speed, (float)speed - correction);

        float speed_left = polarity_switch_left ? -fwd_left : fwd_left;
        float speed_right = polarity_switch_right ? -fwd_right : fwd_right;

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
            gyroSetAngle(target_heading);
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

/**
 * \brief Alternativa k jed_a_sbirej_kostky_buttons() doplněná o gyroskop.
 *        Drží přímý směr target_heading; jakmile sepne první tlačítko u zdi, uvolní gyro
 *        korekci pro dotlačení druhé strany, a po sepnutí OBOU tlačítek vynuluje drift gyra
 *        přesně na globální úhel stěny (0°, 270°, 180°, 90°).
 */
inline void jed_a_sbirej_kostky_buttons_gyro(float target_heading = global_heading) {
    auto& man = rb::Manager::get();
    int speed = 30;
    int mm = 7 * 700;
    int posbyrane_kostky = 0;

    float kp_gyro = 1.2f;
    float m_min_speed = 18.0f;
    float m_max_correction = 8.5f;

    man.motor(left_id).setCurrentPosition(0);
    man.motor(right_id).setCurrentPosition(0);

    float left_pos = 0;
    float right_pos = 0;
    bool left_done = false;
    bool right_done = false;

    unsigned long start_time = millis();
    int timeoutMs = 30000 * (mm / 400);

    otevri_prepazku();
    delay(500);

    while (millis() - start_time < timeoutMs) {
        man.motor(left_id).requestInfo([&](rb::Motor& info) {
            left_pos = info.position();
        });
        man.motor(right_id).requestInfo([&](rb::Motor& info) {
            right_pos = info.position();
        });
        delay(10);

        float correction = 0.0f;
        if (!left_done && !right_done) {
            float heading_error = gyroAngleDiff(target_heading, gyroGetAngle());
            correction = heading_error * kp_gyro;
            correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        }

        float fwd_left = std::max(m_min_speed, (float)speed + correction);
        float fwd_right = std::max(m_min_speed, (float)speed - correction);

        float speed_left = polarity_switch_left ? -fwd_left : fwd_left;
        float speed_right = polarity_switch_right ? -fwd_right : fwd_right;

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
            gyroSetAngle(target_heading);
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

/**
 * \brief Asynchronní alternativa k jed_a_sbirej_kostky_buttons_fast() s gyroskopem
 *        a nulováním odchylky gyra o přední tlačítka na zdi.
 */
inline void jed_a_sbirej_kostky_buttons_fast_gyro(float target_heading = global_heading) {
    auto& man = rb::Manager::get();
    int speed = 30;
    int mm = 7 * 700;
    int posbyrane_kostky = 0;

    float kp_gyro = 1.2f;
    float m_min_speed = 18.0f;
    float m_max_correction = 8.5f;

    man.motor(left_id).setCurrentPosition(0);
    man.motor(right_id).setCurrentPosition(0);

    float left_pos = 0;
    float right_pos = 0;
    bool left_done = false;
    bool right_done = false;

    unsigned long start_time = millis();
    int timeoutMs = 30000 * (mm / 400);

    otevri_prepazku();
    delay(500);

    while (millis() - start_time < timeoutMs) {
        man.motor(left_id).requestInfo([&](rb::Motor& info) {
            left_pos = info.position();
        });
        man.motor(right_id).requestInfo([&](rb::Motor& info) {
            right_pos = info.position();
        });
        delay(10);

        float correction = 0.0f;
        if (!left_done && !right_done) {
            float heading_error = gyroAngleDiff(target_heading, gyroGetAngle());
            correction = heading_error * kp_gyro;
            correction = std::max(-m_max_correction, std::min(correction, m_max_correction));
        }

        float fwd_left = std::max(m_min_speed, (float)speed + correction);
        float fwd_right = std::max(m_min_speed, (float)speed - correction);

        float speed_left = polarity_switch_left ? -fwd_left : fwd_left;
        float speed_right = polarity_switch_right ? -fwd_right : fwd_right;

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
            gyroSetAngle(target_heading);
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
