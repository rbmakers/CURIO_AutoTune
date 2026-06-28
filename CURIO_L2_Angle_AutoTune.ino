#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"

// ==========================================
// 硬體接腳與參數定義
// ==========================================
// 馬達接腳 (X型混控)
const int M1_PIN = 0;   // 右前 (CW)
const int M2_PIN = 11;  // 右後 (CCW)
const int M3_PIN = 14;  // 左後 (CW)
const int M4_PIN = 28;  // 左前 (CCW)

// 狀態 LED 接腳
const int LED_A = 7;    // 狀態指示
const int LED_B = 8;    // 心跳燈
const int LED_C = 9;    // 錯誤/校正指示

// I2C 腳位與 BMI088 位址
const int I2C0_SDA = 20;
const int I2C0_SCL = 25;
#define BMI088_ACC_ADDR  0x18 
#define BMI088_GYRO_ADDR 0x69

// PWM 參數
const int PWM_FREQ = 20000;   // 20kHz 避免尖叫聲
const int PWM_RANGE = 255;    // 8-bit 解析度
const int MOTOR_MAX_LIMIT = 216; // 216/255 = 85% 物理電流防護限制 (防耐流2A接頭燒毀)
const int THROTTLE_BASE = 100;   // 基礎平衡測試油門 (可依測試架配重調整)

// ==========================================
// 📐 姿態與 PID 變數
// ==========================================
BMI088 bmi088(Wire, BMI088_ACC_ADDR, BMI088_GYRO_ADDR);
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Mahony 濾波器參數與四元數
#define kp_mahony 2.0f
#define ki_mahony 0.005f
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;
unsigned long lastUpdate = 0;

// 目前姿態 (歐拉角)
float roll = 0.0f, pitch = 0.0f, yaw_rate = 0.0f;

// 單環 PID 目標值 (Level 1 固定為水平)
float target_roll = 0.0f;
float target_pitch = 0.0f;
float target_yaw_rate = 0.0f; // Yaw 採角速度控制以防鎖死

// PID 增益參數 (此為初始調校值；自動調校完成後會被直接覆寫並即時套用)
float pid_p_roll = 1.2f,  pid_i_roll = 0.05f, pid_d_roll = 0.15f;
float pid_p_pitch = 1.2f, pid_i_pitch = 0.05f, pid_d_pitch = 0.15f;
float pid_p_yaw = 1.5f; // Yaw 單純比例控制速度

// PID 內部誤差變數
float err_prev_roll = 0,  int_roll = 0;
float err_prev_pitch = 0, int_pitch = 0;
unsigned long last_pid_time = 0;
bool is_armed = false;

// ==========================================
// 🎛️ PID 自動調校與規則定義
// ==========================================
const float RELAY_AMPLITUDE   = 40.0f; // 繼電器輸出振幅 d (PWM count, 0~255)
const float RELAY_HYSTERESIS  = 0.6f;  // 誤差遲滯區 ε (度)，需遠小於預期振盪振幅 a
const int   AUTOTUNE_SKIP_CYCLES = 2;  // 捨棄前 N 個週期 (尚未進入穩態極限振盪)
const int   AUTOTUNE_AVG_CYCLES  = 6;  // 取後續 N 個週期平均，降低量測噪訊影響
const unsigned long AUTOTUNE_TIMEOUT_MS = 15000; // 單軸最長測試時間，逾時視為失敗並中止保護

// 調校規則列舉
enum TuningRule { 
    ZN_CLASSIC_PID, 
    ZN_PESSEN_INTEGRAL, 
    ZN_SOME_OVERSHOOT, 
    ZN_NO_OVERSHOOT,
    TYREUS_LUYBEN_PID    // Tyreus-Luyben 規則，追求高穩定裕度、無過衝
};

// 改用 TYREUS_LUYBEN_PID 以獲得更平滑、高穩定裕度的控制響應
const TuningRule AUTOTUNE_RULE = TYREUS_LUYBEN_PID;

enum SystemMode {
    MODE_NORMAL = 0,
    MODE_AUTOTUNE_ROLL,
    MODE_AUTOTUNE_PITCH,
};
SystemMode current_mode = MODE_NORMAL;
bool autotune_chain_to_pitch = false; // true: Roll完成後自動接著做Pitch

struct RelayTuneState {
    int8_t relay_dir        = 1;       // 目前繼電器輸出方向：+1 或 -1
    unsigned long start_time      = 0;
    unsigned long last_switch_time = 0;
    int    cycle_count      = 0;       // 已偵測到的「負轉正」切換次數
    int    valid_cycles     = 0;       // 已採用 (跳過暖機週期後) 的週期數
    float  cycle_min        = 0.0f;
    float  cycle_max        = 0.0f;
    float  sum_period_s     = 0.0f;
    float  sum_amplitude    = 0.0f;
    bool   done             = false;
    bool   timed_out        = false;
};
RelayTuneState rt_roll;
RelayTuneState rt_pitch;

// ==========================================
// 🕒 核心功能函數
// ==========================================
void calibrateSensors() {
    digitalWrite(LED_C, HIGH);
    Serial.println("正在校正感測器，請保持測試架靜止...");
    long samples = 500;
    for(int i=0; i<samples; i++) {
        bmi088.getAcceleration(&ax, &ay, &az);
        bmi088.getGyroscope(&gx, &gy, &gz);
        ax_offset += ax; ay_offset += ay; az_offset += (az - 9.80665f); // 扣除標準重力
        gx_offset += gx; gy_offset += gy; gz_offset += gz;
        delay(4);
    }
    ax_offset /= samples; ay_offset /= samples; az_offset /= samples;
    gx_offset /= samples; gy_offset /= samples; gz_offset /= samples;
    digitalWrite(LED_C, LOW);
    Serial.println("✅ 校正完成！");
}

void updateMahony(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    // 單位化加速度
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // 估計重力方向
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 誤差向量 (外積)
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 積分項
        if(ki_mahony > 0.0f) {
            eIntX += halfex * ki_mahony * dt;
            eIntY += halfey * ki_mahony * dt;
            eIntZ += halfez * ki_mahony * dt;
            gx += eIntX; gy += eIntY; gz += eIntZ;
        }
        // 比例項
        gx += halfex * kp_mahony;
        gy += halfey * kp_mahony;
        gz += halfez * kp_mahony;
    }

    // 更新四元數 (一階龍格庫塔)
    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 單位化四元數
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    // 轉換為歐拉角 (弧度轉角度)
    roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
    pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
}

// ------------------------------------------
// 繼電器回授自動調校 相關函數
// ------------------------------------------
void resetRelayState(RelayTuneState &rt, float current_angle) {
    rt.relay_dir = 1;
    rt.start_time = millis();
    rt.last_switch_time = 0;
    rt.cycle_count = 0;
    rt.valid_cycles = 0;
    rt.cycle_min = current_angle;
    rt.cycle_max = current_angle;
    rt.sum_period_s = 0.0f;
    rt.sum_amplitude = 0.0f;
    rt.done = false;
    rt.timed_out = false;
}

float relayStep(RelayTuneState &rt, float measured_angle, float target_angle, unsigned long now_ms) {
    float error = target_angle - measured_angle;

    // --- 持續追蹤本週期內角度最大/最小值 (用於振幅計算) ---
    if (measured_angle > rt.cycle_max) rt.cycle_max = measured_angle;
    if (measured_angle < rt.cycle_min) rt.cycle_min = measured_angle;

    // --- 繼電器切換邏輯 (含遲滯區 ε) ---
    if (rt.relay_dir < 0 && error > RELAY_HYSTERESIS) {
        rt.relay_dir = +1;
        if (rt.last_switch_time != 0) {
            float period_s = (now_ms - rt.last_switch_time) / 1000.0f;
            rt.cycle_count++;

            if (rt.cycle_count > AUTOTUNE_SKIP_CYCLES) {
                float amplitude = (rt.cycle_max - rt.cycle_min) / 2.0f;
                rt.sum_period_s  += period_s;
                rt.sum_amplitude += amplitude;
                rt.valid_cycles++;
            }
        }
        rt.last_switch_time = now_ms;
        rt.cycle_min = measured_angle;
        rt.cycle_max = measured_angle;
    } else if (rt.relay_dir > 0 && error < -RELAY_HYSTERESIS) {
        rt.relay_dir = -1;
    }

    if (rt.valid_cycles >= AUTOTUNE_AVG_CYCLES) rt.done = true;
    if (now_ms - rt.start_time > AUTOTUNE_TIMEOUT_MS) rt.timed_out = true;
    return rt.relay_dir * RELAY_AMPLITUDE;
}

// 由量得的 Ku, Pu 依選定規則換算 PID 三項增益
void computeGainsFromRelay(float Ku, float Pu, float &Kp, float &Ki, float &Kd) {
    float Ti = 0, Td = 0;
    switch (AUTOTUNE_RULE) {
        case ZN_CLASSIC_PID:
            Kp = 0.60f * Ku;
            Ti = 0.50f * Pu;  Td = 0.125f * Pu;
            break;
        case ZN_PESSEN_INTEGRAL:
            Kp = 0.70f * Ku;
            Ti = 0.40f * Pu;  Td = 0.150f * Pu;
            break;
        case ZN_SOME_OVERSHOOT:
            Kp = 0.33f * Ku;
            Ti = 0.50f * Pu;  Td = 0.333f * Pu;
            break;
        case ZN_NO_OVERSHOOT:
            Kp = 0.20f * Ku;
            Ti = 0.50f * Pu;  Td = 0.333f * Pu;
            break;
        case TYREUS_LUYBEN_PID:
        default:
            // 引入 Tyreus-Luyben 公式：追求更平滑、高穩定裕度的控制
            Kp = Ku / 3.2f;        // 降低比例增益提供充足的安全裕度 (Kp = 0.3125 * Ku)
            Ti = 2.20f * Pu;       // 大幅拉長積分時間，有效消除過衝
            Td = Pu / 6.5f;        // 溫和的微分時間 (Td ≈ 0.1538 * Pu)，抑制高頻噪訊
            break;
    }
    Ki = Kp / Ti;
    Kd = Kp * Td;
}

void printAutotuneResult(const char* axis_name, float Ku, float Pu, float Kp, float Ki, float Kd) {
    Serial.println("==========================================");
    Serial.print("✅ ["); Serial.print(axis_name); Serial.println("] 軸自動調校完成");
    Serial.print("  極限增益 Ku = "); Serial.println(Ku, 4);
    Serial.print("  極限週期 Pu = "); Serial.print(Pu, 4); Serial.println(" s");
    Serial.print("  -> Kp = "); Serial.println(Kp, 4);
    Serial.print("  -> Ki = "); Serial.println(Ki, 4);
    Serial.print("  -> Kd = "); Serial.println(Kd, 4);
    Serial.println("  (已即時套用；請手動將數值寫回程式碼以永久保存)");
    Serial.println("==========================================");
}

void finalizeAxisTuning(RelayTuneState &rt, const char* axis_name,
                         float &out_p, float &out_i, float &out_d) {
    float Pu = rt.sum_period_s / rt.valid_cycles;
    float a  = rt.sum_amplitude / rt.valid_cycles;

    if (a < 0.1f) {
        Serial.println("⚠️ 警告：偵測到的振盪振幅過小，調校結果可能不可靠！");
        a = 0.1f; // 避免除以零
    }

    float Ku = (4.0f * RELAY_AMPLITUDE) / (PI * a);
    float Kp, Ki, Kd;
    computeGainsFromRelay(Ku, Pu, Kp, Ki, Kd);
    printAutotuneResult(axis_name, Ku, Pu, Kp, Ki, Kd);

    out_p = Kp;
    out_i = Ki; out_d = Kd;
}

void startAutotuneSequence() {
    Serial.println("🚀 開始完整自動調校流程：Roll -> Pitch");
    autotune_chain_to_pitch = true;
    resetRelayState(rt_roll, roll);
    current_mode = MODE_AUTOTUNE_ROLL;
}

void startAutotuneRollOnly() {
    Serial.println("🚀 開始 Roll 軸自動調校...");
    autotune_chain_to_pitch = false;
    resetRelayState(rt_roll, roll);
    current_mode = MODE_AUTOTUNE_ROLL;
}

void startAutotunePitchOnly() {
    Serial.println("🚀 開始 Pitch 軸自動調校...");
    resetRelayState(rt_pitch, pitch);
    current_mode = MODE_AUTOTUNE_PITCH;
}

void abortAutotune(const char* reason) {
    Serial.print("⛔ 自動調校中止：");
    Serial.println(reason);
    current_mode = MODE_NORMAL;
    int_roll = 0; err_prev_roll = 0; // 強制歸零積分項與微分舊誤差
    int_pitch = 0; err_prev_pitch = 0;
}

// ==========================================
// 🚀 Arduino 核心 Setup & Loop
// ==========================================
void setup() {
    Serial.begin(115200);
    
    pinMode(M1_PIN, OUTPUT); pinMode(M2_PIN, OUTPUT);
    pinMode(M3_PIN, OUTPUT); pinMode(M4_PIN, OUTPUT);
    pinMode(LED_A, OUTPUT);  pinMode(LED_B, OUTPUT); pinMode(LED_C, OUTPUT);

    analogWriteFreq(PWM_FREQ);
    analogWriteRange(PWM_RANGE);
    
    // 初始馬達關閉
    analogWrite(M1_PIN, 0);
    analogWrite(M2_PIN, 0);
    analogWrite(M3_PIN, 0); analogWrite(M4_PIN, 0);

    // 初始化 I2C0
    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.begin();
    while (!bmi088.isConnection()) {
        Serial.println("❌ BMI088 連線失敗，檢查硬體中...");
        digitalWrite(LED_C, HIGH); delay(500); digitalWrite(LED_C, LOW); delay(500);
    }
    bmi088.initialize();
    
    // 進行靜止校正
    calibrateSensors();

    // 安全檢查
    if (RELAY_AMPLITUDE >= THROTTLE_BASE || RELAY_AMPLITUDE >= (MOTOR_MAX_LIMIT - THROTTLE_BASE)) {
        Serial.println("⚠️ 警告: RELAY_AMPLITUDE 設定可能造成 PWM 飽和，調校結果將不準確！");
    }

    Serial.println("==========================================");
    Serial.println(" PID 自動調校 (繼電器回授法) 指令說明");
    Serial.println("   t = 全自動調校 (Roll -> Pitch)");
    Serial.println("   r = 僅調校 Roll 軸");
    Serial.println("   p = 僅調校 Pitch 軸");
    Serial.println("   x = 緊急中止調校 (隨時可用)");
    Serial.println("==========================================");
    
    lastUpdate = micros();
    last_pid_time = millis();
    is_armed = true; // 校正完成後直接解鎖進入固定平衡測試
    digitalWrite(LED_A, HIGH);
}

void loop() {
    // 0. 讀取序列指令
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'x' || cmd == 'X') {
            abortAutotune("使用者手動中止");
        } else if (current_mode == MODE_NORMAL && is_armed) {
            if (cmd == 't' || cmd == 'T') startAutotuneSequence();
            else if (cmd == 'r' || cmd == 'R') startAutotuneRollOnly();
            else if (cmd == 'p' || cmd == 'P') startAutotunePitchOnly();
        }
    }

    // 1. 讀取感測器數據
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);

    // 扣除校正偏置
    ax -= ax_offset; ay -= ay_offset; az -= az_offset;
    gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

    // 2. 姿態更新 (Mahony)
    unsigned long now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f; // 防呆

    // 弧度轉換
    float gx_rad = gx * 0.0174533f;
    float gy_rad = gy * 0.0174533f;
    float gz_rad = gz * 0.0174533f;
    updateMahony(ax, ay, az, gx_rad, gy_rad, gz_rad, dt);

    // 3. 核心 PID 控制迴路 (固定控制頻率：500Hz / 2ms)
    unsigned long currentTime = millis();
    float pid_dt = (currentTime - last_pid_time) / 1000.0f;
    
    if (pid_dt >= 0.002f) {
        last_pid_time = currentTime;

        // --- 安全防護：過度傾斜自動斷電 ---
        if (abs(roll) > 60.0f || abs(pitch) > 60.0f) {
            if (is_armed) {
                is_armed = false;
                digitalWrite(LED_A, LOW);
                digitalWrite(LED_C, HIGH);
                if (current_mode != MODE_NORMAL) {
                    abortAutotune("傾斜角度超過防護門檻 (60°)");
                }
            }
        }

        float output_roll = 0, output_pitch = 0, output_yaw = 0;
        if (is_armed) {
            // Yaw 角速度阻尼
            float err_yaw_rate = target_yaw_rate - gz;
            output_yaw = pid_p_yaw * err_yaw_rate;

            if (current_mode == MODE_AUTOTUNE_ROLL) {
                // Roll 軸：繼電器輸出
                output_roll = relayStep(rt_roll, roll, target_roll, currentTime);

                // Pitch 軸：維持原本單環 PID 穩定水平
                float err_pitch = target_pitch - pitch;
                int_pitch += err_pitch * pid_dt;
                int_pitch = constrain(int_pitch, -50, 50);
                float deriv_pitch = (err_pitch - err_prev_pitch) / pid_dt;
                output_pitch = (pid_p_pitch * err_pitch) + (pid_i_pitch * int_pitch) + (pid_d_pitch * deriv_pitch);
                err_prev_pitch = err_pitch;

                if (rt_roll.done) {
                    finalizeAxisTuning(rt_roll, "ROLL", pid_p_roll, pid_i_roll, pid_d_roll);
                    int_roll = 0; err_prev_roll = 0;
                    if (autotune_chain_to_pitch) {
                        Serial.println("➡️  接續進行 Pitch 軸繼電器測試...");
                        resetRelayState(rt_pitch, pitch);
                        current_mode = MODE_AUTOTUNE_PITCH;
                    } else {
                        current_mode = MODE_NORMAL;
                    }
                } else if (rt_roll.timed_out) {
                    abortAutotune("Roll 軸逾時，未偵測到穩定振盪");
                }

            } else if (current_mode == MODE_AUTOTUNE_PITCH) {
                // Pitch 軸：繼電器輸出
                output_pitch = relayStep(rt_pitch, pitch, target_pitch, currentTime);

                // Roll 軸：維持原本單環 PID 穩定水平
                float err_roll = target_roll - roll;
                int_roll += err_roll * pid_dt;
                int_roll = constrain(int_roll, -50, 50);
                float deriv_roll = (err_roll - err_prev_roll) / pid_dt;
                output_roll = (pid_p_roll * err_roll) + (pid_i_roll * int_roll) + (pid_d_roll * deriv_roll);
                err_prev_roll = err_roll;

                if (rt_pitch.done) {
                    finalizeAxisTuning(rt_pitch, "PITCH", pid_p_pitch, pid_i_pitch, pid_d_pitch);
                    int_pitch = 0; err_prev_pitch = 0;
                    Serial.println("🎉 自動調校流程結束，新增益已套用於目前運行中。");
                    current_mode = MODE_NORMAL;
                } else if (rt_pitch.timed_out) {
                    abortAutotune("Pitch 軸逾時，未偵測到穩定振盪");
                }

            } else {
                // MODE_NORMAL：常規單環 PID 控制
                // Roll PID
                float err_roll = target_roll - roll;
                int_roll += err_roll * pid_dt;
                int_roll = constrain(int_roll, -50, 50);
                float deriv_roll = (err_roll - err_prev_roll) / pid_dt;
                output_roll = (pid_p_roll * err_roll) + (pid_i_roll * int_roll) + (pid_d_roll * deriv_roll);
                err_prev_roll = err_roll;

                // Pitch PID
                float err_pitch = target_pitch - pitch;
                int_pitch += err_pitch * pid_dt;
                int_pitch = constrain(int_pitch, -50, 50);
                float deriv_pitch = (err_pitch - err_prev_pitch) / pid_dt;
                output_pitch = (pid_p_pitch * err_pitch) + (pid_i_pitch * int_pitch) + (pid_d_pitch * deriv_pitch);
                err_prev_pitch = err_pitch;
            }
        }

        // 4. 馬達混控 (X型混控架構)
        int m1 = THROTTLE_BASE - output_roll + output_pitch + output_yaw;
        int m2 = THROTTLE_BASE - output_roll - output_pitch - output_yaw;
        int m3 = THROTTLE_BASE + output_roll - output_pitch + output_yaw;
        int m4 = THROTTLE_BASE + output_roll + output_pitch - output_yaw;

        // 5. 輸出限幅與防護
        if (is_armed) {
            m1 = constrain(m1, 0, MOTOR_MAX_LIMIT);
            m2 = constrain(m2, 0, MOTOR_MAX_LIMIT);
            m3 = constrain(m3, 0, MOTOR_MAX_LIMIT);
            m4 = constrain(m4, 0, MOTOR_MAX_LIMIT);
        } else {
            m1 = m2 = m3 = m4 = 0;
        }

        // 寫入 PWM
        analogWrite(M1_PIN, m1);
        analogWrite(M2_PIN, m2);
        analogWrite(M3_PIN, m3);
        analogWrite(M4_PIN, m4);
    }

    // 6. 系統心跳燈與資料輸出 (500ms 週期)
    static unsigned long last_blink = 0;
    if (millis() - last_blink > 500) {
        last_blink = millis();
        digitalWrite(LED_B, !digitalRead(LED_B));
        
        Serial.print("Roll:"); Serial.print(roll); Serial.print(",");
        Serial.print("Pitch:"); Serial.print(pitch);
        if (current_mode == MODE_AUTOTUNE_ROLL) {
            Serial.print(",Tuning:ROLL cycles=");
            Serial.print(rt_roll.valid_cycles);
            Serial.print("/"); Serial.print(AUTOTUNE_AVG_CYCLES);
        } else if (current_mode == MODE_AUTOTUNE_PITCH) {
            Serial.print(",Tuning:PITCH cycles=");
            Serial.print(rt_pitch.valid_cycles);
            Serial.print("/"); Serial.print(AUTOTUNE_AVG_CYCLES);
        }
        Serial.println();
    }
}