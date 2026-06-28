#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"

// ==========================================
// 🛠️ 硬體接腳與參數定義
// ==========================================
// 馬達接腳 (X型混控)
const int M1_PIN = 0;   // 右前 (CW)
const int M2_PIN = 11;  // 右後 (CCW)
const int M3_PIN = 14;  // 左後 (CW)
const int M4_PIN = 28;  // 左前 (CCW)

// 狀態 LED 接腳
const int LED_A = 7;    // 解鎖狀態指示
const int LED_B = 8;    // 心跳燈
const int LED_C = 9;    // 錯誤/校正指示

// I2C 腳位與 BMI088 位址
const int I2C0_SDA = 20;
const int I2C0_SCL = 25;
#define BMI088_ACC_ADDR  0x18
#define BMI088_GYRO_ADDR 0x69

// PWM 參數
const int PWM_FREQ = 20000;      // 20kHz 避免高頻噪訊與馬達尖叫
const int PWM_RANGE = 255;       // 8-bit 解析度
const int MOTOR_MAX_LIMIT = 216; // 216/255 = 85% 物理電流防護限制 (防耐流2A接頭燒毀)
const int THROTTLE_BASE = 100;   // 基礎平衡測試油門 (可依測試架配重調整)

// ==========================================
// 📐 姿態與串級雙環 PID 變數
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

// 目前姿態 (歐拉角與角速度)
float roll = 0.0f, pitch = 0.0f, yaw_rate = 0.0f;

// 串級雙環控制目標值
float target_roll = 0.0f;
float target_pitch = 0.0f;
float target_yaw_rate = 0.0f; // Yaw 採角速度控制以防鎖死

// --- 雙環 PID 參數變數 ---
// 1. 外環 (Angle Loop - 角度控制，純 P) -> 輸出為期望角速度 (deg/s)
float kP_angle_roll = 4.5f,  kI_angle_roll = 0.0f,  kD_angle_roll = 0.0f;
float kP_angle_pitch = 4.5f, kI_angle_pitch = 0.0f, kD_angle_pitch = 0.0f;
// 2. 內環 (Rate Loop - 角速度控制，完整 PID) -> 輸出為馬達修正量 (PWM count)
//    此為初始調校值；自動調校完成後會直接覆寫並即時套用
float kP_rate_roll = 0.8f,   kI_rate_roll = 0.04f,  kD_rate_roll = 0.01f;
float kP_rate_pitch = 0.8f,  kI_rate_pitch = 0.04f, kD_rate_pitch = 0.01f;
float kP_rate_yaw = 1.5f,    kI_rate_yaw = 0.02f,   kD_rate_yaw = 0.0f;

// 雙環誤差與積分歷史 (角度外環目前為純 P，故 int_angle_* 暫未使用，保留供未來擴充)
float err_prev_angle_roll = 0, int_angle_roll = 0;
float err_prev_angle_pitch = 0, int_angle_pitch = 0;
float err_prev_rate_roll = 0, int_rate_roll = 0;
float err_prev_rate_pitch = 0, int_rate_pitch = 0;
float err_prev_rate_yaw = 0, int_rate_yaw = 0;

unsigned long last_pid_time = 0;
bool is_armed = false;

// ==========================================
// 🎛️ PID 自動調校（繼電器回授法 / Relay Feedback）
// ==========================================
// 原理（Åström–Hägglund Relay Feedback Method, 1984）：
//
//   暫時將內環 PID 控制器替換成一個「繼電器」(Relay)：輸出只有 +d 與 -d
//   兩個狀態，依誤差正負切換（並加入遲滯區 ε 抑制感測器噪訊造成的高頻誤切換）。
//   此非線性回授會強迫受控系統產生穩定的「極限振盪」(Limit Cycle)。
//
//   在 ε ≪ a（遲滯區遠小於振盪振幅）的近似下，由描述函數法可推得：
//
//       ωu = 2π / Pu              (極限角頻率，由實測振盪週期 Pu 取得)
//       Ku ≈ 4d / (π · a)         (極限增益，由繼電器振幅 d 與實測振盪振幅 a 取得)
//
//   ⚠️ 架構說明（比照 CURIO_L2_Dual_AutoTune.ino 的雙環調校方式）：
//   調校期間外環與內環都被繼電器「整段跳過」，繼電器直接驅動最終的
//   output_roll/output_pitch（等同於對「PWM → 角度」這個含兩次積分的完整
//   受控體做測試），但量得的 Ku、Pu 換算出的增益會寫入「內環」(Rate Loop)
//   的 Kp/Ki/Kd。這與 L2_Dual 採用的設計完全一致，但嚴格而言內環真正對應
//   的受控體是「PWM → 角速度」(少一次積分)，兩者頻域特性不同，請將調校
//   結果視為一組合理的起始值，仍建議搭配實際飛行/台架測試手動微調。
//
//   參考文獻：
//   Åström, K. J., & Hägglund, T. (1984). Automatic tuning of simple
//   regulators with specifications on phase and amplitude margins.
//   Automatica, 20(5), 645–651.

const float RELAY_AMPLITUDE   = 40.0f;  // 繼電器輸出振幅 d (PWM count, 0~255)
const float RELAY_HYSTERESIS  = 0.6f;    // 誤差遲滯區 ε (度)
const int   AUTOTUNE_SKIP_CYCLES = 2;    // 捨棄前 N 個週期 (尚未進入穩態極限振盪)
const int   AUTOTUNE_AVG_CYCLES  = 6;    // 取後續 N 個週期平均，降低量測噪訊影響
const unsigned long AUTOTUNE_TIMEOUT_MS = 15000; // 單軸最長測試時間，逾時視為失敗並中止保護

// Ziegler–Nichols 極限增益法，四種經典規則可選（L1 系列維持原始 ZN 規則，
// 與 L2 系列改用 TYREUS_LUYBEN_PID 的選擇互為對照）
enum TuningRule { ZN_CLASSIC_PID, ZN_PESSEN_INTEGRAL, ZN_SOME_OVERSHOOT, ZN_NO_OVERSHOOT };

// ⚠️ 建議：測試架上第一次自動調校請使用 ZN_NO_OVERSHOOT (較保守、不易劇烈擺動)
const TuningRule AUTOTUNE_RULE = ZN_NO_OVERSHOOT;

enum SystemMode {
    MODE_NORMAL = 0,
    MODE_AUTOTUNE_ROLL,
    MODE_AUTOTUNE_PITCH,
};
SystemMode current_mode = MODE_NORMAL;
bool autotune_chain_to_pitch = false; // true: Roll完成後自動接著做Pitch

struct RelayTuneState {
    int8_t relay_dir        = 1;     // 目前繼電器輸出方向：+1 或 -1
    unsigned long start_time      = 0;
    unsigned long last_switch_time = 0;
    int    cycle_count      = 0;     // 已偵測到的「負轉正」切換次數
    int    valid_cycles     = 0;     // 已採用 (跳過暖機週期後) 的週期數
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
// 🕒 核心功能與感測器函數
// ==========================================
void calibrateSensors() {
    digitalWrite(LED_C, HIGH);
    Serial.println("正在校正感測器，請保持測試架靜止...");
    long samples = 500;
    for (int i = 0; i < samples; i++) {
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
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
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
        if (ki_mahony > 0.0f) {
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

    // 更新四元數 (一階龍格庫塔；本函式輸入的 gx/gy/gz 必須是「弧度/秒」)
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

// 執行一次繼電器回授步驟，回傳本次應輸出的修正量 (PWM count)
// measured_angle: 目前角度(度)，target_angle: 目標角度(固定為 0)
float relayStep(RelayTuneState &rt, float measured_angle, float target_angle, unsigned long now_ms) {
    float error = target_angle - measured_angle;

    // --- 持續追蹤本週期內角度最大/最小值 (用於振幅計算) ---
    if (measured_angle > rt.cycle_max) rt.cycle_max = measured_angle;
    if (measured_angle < rt.cycle_min) rt.cycle_min = measured_angle;

    // --- 繼電器切換邏輯 (含遲滯區 ε，避免噪訊造成高頻誤切換) ---
    // 以「負轉正」邊作為一個完整週期的起點/終點
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

// 由量得的 Ku, Pu 依選定規則換算「內環」三項增益
void computeGainsFromRelay(float Ku, float Pu, float &Kp, float &Ki, float &Kd) {
    float Ti = 0, Td = 0;
    switch (AUTOTUNE_RULE) {
        case ZN_CLASSIC_PID:
            Kp = 0.60f * Ku;  Ti = 0.50f * Pu;  Td = 0.125f * Pu;
            break;
        case ZN_PESSEN_INTEGRAL:
            Kp = 0.70f * Ku;  Ti = 0.40f * Pu;  Td = 0.150f * Pu;
            break;
        case ZN_SOME_OVERSHOOT:
            Kp = 0.33f * Ku;  Ti = 0.50f * Pu;  Td = 0.333f * Pu;
            break;
        case ZN_NO_OVERSHOOT:
        default:
            Kp = 0.20f * Ku;  Ti = 0.50f * Pu;  Td = 0.333f * Pu;
            break;
    }
    Ki = Kp / Ti;
    Kd = Kp * Td;
}

void printAutotuneResult(const char* axis_name, float Ku, float Pu, float Kp, float Ki, float Kd) {
    Serial.println("==========================================");
    Serial.print("✅ [串級雙環 - "); Serial.print(axis_name); Serial.println(" 速率內環] 自動調校完成");
    Serial.print("  極限增益 Ku = "); Serial.println(Ku, 4);
    Serial.print("  極限週期 Pu = "); Serial.print(Pu, 4); Serial.println(" s");
    Serial.print("  -> Rate Kp = "); Serial.println(Kp, 4);
    Serial.print("  -> Rate Ki = "); Serial.println(Ki, 4);
    Serial.print("  -> Rate Kd = "); Serial.println(Kd, 4);
    Serial.println("  (已即時套用至 Rate 內環；外環 Kp 為固定值，可依反應靈敏度手動增減)");
    Serial.println("==========================================");
}

// 由 rt 的統計結果計算 Ku/Pu，換算增益並直接寫入傳入的「內環」PID 變數
void finalizeAxisTuning(RelayTuneState &rt, const char* axis_name,
                         float &out_p, float &out_i, float &out_d) {
    float Pu = rt.sum_period_s / rt.valid_cycles;
    float a  = rt.sum_amplitude / rt.valid_cycles;

    if (a < 0.1f) {
        Serial.println("⚠️ 警告：偵測到的振盪振幅過小，調校結果可能不可靠！");
        Serial.println("   (請嘗試增加 RELAY_AMPLITUDE 或檢查測試架是否卡住後重試)");
        a = 0.1f; // 避免除以零
    }

    float Ku = (4.0f * RELAY_AMPLITUDE) / (PI * a);
    float Kp, Ki, Kd;
    computeGainsFromRelay(Ku, Pu, Kp, Ki, Kd);
    printAutotuneResult(axis_name, Ku, Pu, Kp, Ki, Kd);

    out_p = Kp; out_i = Ki; out_d = Kd;
}

void startAutotuneSequence() {
    Serial.println("🚀 開始完整雙環調校流程：Roll 內環 -> Pitch 內環 (外環維持比例控制)");
    autotune_chain_to_pitch = true;
    resetRelayState(rt_roll, roll);
    current_mode = MODE_AUTOTUNE_ROLL;
}

void abortAutotune(const char* reason) {
    Serial.print("⛔ 自動調校中止："); Serial.println(reason);
    current_mode = MODE_NORMAL;
    is_armed = false;            // 中止時安全斷電
    digitalWrite(LED_A, LOW);
    // 歸零內環積分項與微分歷史，避免重新解鎖時殘留誤差造成輸出突變
    int_rate_roll = 0;  err_prev_rate_roll = 0;
    int_rate_pitch = 0; err_prev_rate_pitch = 0;
    int_rate_yaw = 0;   err_prev_rate_yaw = 0;
}

// ==========================================
// 🚀 Arduino 核心 Setup & Loop
// ==========================================
void setup() {
    Serial.begin(115200);

    pinMode(LED_A, OUTPUT); pinMode(LED_B, OUTPUT); pinMode(LED_C, OUTPUT);
    digitalWrite(LED_A, LOW); digitalWrite(LED_B, LOW); digitalWrite(LED_C, LOW);
    pinMode(M1_PIN, OUTPUT); pinMode(M2_PIN, OUTPUT);
    pinMode(M3_PIN, OUTPUT); pinMode(M4_PIN, OUTPUT);

    // 初始化 I2C0 與 BMI088
    Wire.setSDA(I2C0_SDA); Wire.setSCL(I2C0_SCL);
    Wire.begin();
    Wire.setClock(400000); // 400kHz 快速模式

    while (!bmi088.isConnection()) {
        Serial.println("❌ BMI088 連線失敗，檢查硬體中...");
        digitalWrite(LED_C, HIGH); delay(500); digitalWrite(LED_C, LOW); delay(500);
    }
    bmi088.initialize();

    // 進行靜止校正
    calibrateSensors();

    // 自動調校參數安全檢查：避免繼電器振幅造成 PWM 飽和而使量測失真
    if (RELAY_AMPLITUDE >= THROTTLE_BASE || RELAY_AMPLITUDE >= (MOTOR_MAX_LIMIT - THROTTLE_BASE)) {
        Serial.println("⚠️ 警告: RELAY_AMPLITUDE 設定可能造成 PWM 飽和，調校結果將不準確！");
    }

    // 配置高頻 PWM
    analogWriteFreq(PWM_FREQ);
    analogWriteRange(PWM_RANGE);

    // 強制重置所有馬達輸出為 0
    analogWrite(M1_PIN, 0); analogWrite(M2_PIN, 0);
    analogWrite(M3_PIN, 0); analogWrite(M4_PIN, 0);

    Serial.println("==========================================");
    Serial.println(" CURIO 串級雙環自動調校 (L1 / ZN_NO_OVERSHOOT)");
    Serial.println("   G = 解鎖並開始全自動調校 (Roll -> Pitch)");
    Serial.println("   S = 中止調校並安全斷電");
    Serial.println("==========================================");

    lastUpdate = micros();
    last_pid_time = millis();
}

void loop() {
    // 1. 處理外部序列埠指令
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'G' || cmd == 'g') {
            if (!is_armed) {
                is_armed = true;
                digitalWrite(LED_A, HIGH);
                startAutotuneSequence();
            }
        } else if (cmd == 'S' || cmd == 's') {
            abortAutotune("使用者手動中止");
        }
    }

    // 2. 讀取感測器並計算時間增量 (dt)
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);

    // 扣除校正偏置
    ax -= ax_offset; ay -= ay_offset; az -= az_offset;
    gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

    unsigned long now_u = micros();
    float dt = (now_u - lastUpdate) / 1000000.0f;
    lastUpdate = now_u;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f; // 異常保護

    // BMI088::getGyroscope() 回傳值單位為 度/秒，Mahony 濾波器內部以四元數
    // 積分需要 弧度/秒，因此送入前先做單位轉換
    float gx_rad = gx * 0.0174533f;
    float gy_rad = gy * 0.0174533f;
    float gz_rad = gz * 0.0174533f;

    // 透過 Mahony 濾波器即時計算目前姿態角度
    updateMahony(ax, ay, az, gx_rad, gy_rad, gz_rad, dt);

    // 讀取目前的角速度（用於角速度內環控制，單位：deg/s）
    // gx/gy/gz 本身已經是 deg/s，不需要再乘 57.29578
    float rate_roll  = gx;
    float rate_pitch = gy;
    float rate_yaw   = gz;

    // 3. 飛行安全防護機制 (傾角超過 60 度自動斷電保護；任何模式下都不可被繞過)
    if (abs(roll) > 60.0f || abs(pitch) > 60.0f) {
        if (is_armed) {
            digitalWrite(LED_C, HIGH); // 亮起錯誤燈
            abortAutotune("安全防護觸發：傾角超過 60° 防護門檻");
        }
    }

    // 4. 串級雙環 PID 計算與自動調校狀態機 (固定控制頻率：500Hz / 2ms)
    unsigned long now_ms = millis();
    if (now_ms - last_pid_time >= 2) {
        float dt_pid = (now_ms - last_pid_time) / 1000.0f;
        last_pid_time = now_ms;

        float output_roll = 0, output_pitch = 0, output_yaw = 0;

        // 未解鎖時，以下整段運算 (含內環積分項) 完全不執行，避免積分值
        // 在停機/等待期間持續累積漂移，解鎖瞬間造成輸出突變
        if (is_armed) {

            // --- Roll 軸調校與控制 ---
            if (current_mode == MODE_AUTOTUNE_ROLL) {
                // 自動調校時：外環+內環整段跳過，由繼電器直接刺激最終輸出，量測角度振盪
                output_roll = relayStep(rt_roll, roll, target_roll, now_ms);

                if (rt_roll.done) {
                    finalizeAxisTuning(rt_roll, "Roll", kP_rate_roll, kI_rate_roll, kD_rate_roll);
                    int_rate_roll = 0; err_prev_rate_roll = 0;
                    if (autotune_chain_to_pitch) {
                        Serial.println("➡️  接續進行 Pitch 軸繼電器測試...");
                        current_mode = MODE_AUTOTUNE_PITCH;
                        resetRelayState(rt_pitch, pitch);
                    } else {
                        current_mode = MODE_NORMAL;
                    }
                } else if (rt_roll.timed_out) {
                    abortAutotune("Roll 軸逾時，未偵測到穩定振盪 (請檢查 RELAY_AMPLITUDE 或測試架是否卡住)");
                }
            } else {
                // 正常串級雙環模式
                // [外環：角度控制，純 P] -> 輸出期望角速度
                float err_ang_roll = target_roll - roll;
                float target_rate_roll = kP_angle_roll * err_ang_roll;

                // [內環：角速度控制，完整 PID] -> 輸出馬達修正量
                float err_rate_roll = target_rate_roll - rate_roll;
                int_rate_roll += err_rate_roll * dt_pid;
                int_rate_roll = constrain(int_rate_roll, -50.0f, 50.0f); // 積分限幅
                float deriv_rate_roll = (err_rate_roll - err_prev_rate_roll) / dt_pid;
                err_prev_rate_roll = err_rate_roll;

                output_roll = (kP_rate_roll * err_rate_roll) + (kI_rate_roll * int_rate_roll) + (kD_rate_roll * deriv_rate_roll);
            }

            // --- Pitch 軸調校與控制 ---
            if (current_mode == MODE_AUTOTUNE_PITCH) {
                output_pitch = relayStep(rt_pitch, pitch, target_pitch, now_ms);

                if (rt_pitch.done) {
                    finalizeAxisTuning(rt_pitch, "Pitch", kP_rate_pitch, kI_rate_pitch, kD_rate_pitch);
                    int_rate_pitch = 0; err_prev_rate_pitch = 0;
                    current_mode = MODE_NORMAL;
                    Serial.println("🎉 雙環內環自動調校流程結束，新增益已即時套用於目前運行中。");
                } else if (rt_pitch.timed_out) {
                    abortAutotune("Pitch 軸逾時，未偵測到穩定振盪 (請檢查 RELAY_AMPLITUDE 或測試架是否卡住)");
                }
            } else {
                // 正常串級雙環模式
                float err_ang_pitch = target_pitch - pitch;
                float target_rate_pitch = kP_angle_pitch * err_ang_pitch;

                float err_rate_pitch = target_rate_pitch - rate_pitch;
                int_rate_pitch += err_rate_pitch * dt_pid;
                int_rate_pitch = constrain(int_rate_pitch, -50.0f, 50.0f);
                float deriv_rate_pitch = (err_rate_pitch - err_prev_rate_pitch) / dt_pid;
                err_prev_rate_pitch = err_rate_pitch;

                output_pitch = (kP_rate_pitch * err_rate_pitch) + (kI_rate_pitch * int_rate_pitch) + (kD_rate_pitch * deriv_rate_pitch);
            }

            // --- Yaw 軸常規控制 (單環角速度 PID，無論在任何模式下皆持續運作，避免測試架自轉) ---
            float err_rate_yaw = target_yaw_rate - rate_yaw;
            int_rate_yaw += err_rate_yaw * dt_pid;
            int_rate_yaw = constrain(int_rate_yaw, -30.0f, 30.0f);
            float deriv_rate_yaw = (err_rate_yaw - err_prev_rate_yaw) / dt_pid;
            err_prev_rate_yaw = err_rate_yaw;
            output_yaw = (kP_rate_yaw * err_rate_yaw) + (kI_rate_yaw * int_rate_yaw) + (kD_rate_yaw * deriv_rate_yaw);

        } // end if (is_armed)

        // 5. 馬達混控 (X型混控架構)
        // M1(右前,CW), M2(右後,CCW), M3(左後,CW), M4(左前,CCW)
        int m1 = THROTTLE_BASE - output_roll + output_pitch + output_yaw;
        int m2 = THROTTLE_BASE - output_roll - output_pitch - output_yaw;
        int m3 = THROTTLE_BASE + output_roll - output_pitch + output_yaw;
        int m4 = THROTTLE_BASE + output_roll + output_pitch - output_yaw;

        // 6. 輸出限幅與防護 (嚴格限制最大輸出以保護硬體)
        if (is_armed) {
            m1 = constrain(m1, 0, MOTOR_MAX_LIMIT);
            m2 = constrain(m2, 0, MOTOR_MAX_LIMIT);
            m3 = constrain(m3, 0, MOTOR_MAX_LIMIT);
            m4 = constrain(m4, 0, MOTOR_MAX_LIMIT);
        } else {
            m1 = m2 = m3 = m4 = 0; // 未解鎖或觸發防護時強制關閉
        }

        // 寫入 PWM 驅動馬達
        analogWrite(M1_PIN, m1);
        analogWrite(M2_PIN, m2);
        analogWrite(M3_PIN, m3);
        analogWrite(M4_PIN, m4);
    }

    // 7. 系統心跳燈與除錯輸出 (LED_B)
    static unsigned long last_blink = 0;
    if (millis() - last_blink > 500) {
        last_blink = millis();
        digitalWrite(LED_B, !digitalRead(LED_B));

        // 偵錯輸出：可在 Serial Plotter / Monitor 觀察動態響應與調校進度
        Serial.print("Mode:"); Serial.print(current_mode); Serial.print(",");
        Serial.print("Roll:"); Serial.print(roll); Serial.print(",");
        Serial.print("Pitch:"); Serial.print(pitch); Serial.print(",");
        Serial.print("RateRoll:"); Serial.print(rate_roll); Serial.print(",");
        if (current_mode == MODE_AUTOTUNE_ROLL) {
            Serial.print("Tuning:ROLL cycles="); Serial.print(rt_roll.valid_cycles);
            Serial.print("/"); Serial.print(AUTOTUNE_AVG_CYCLES);
        } else if (current_mode == MODE_AUTOTUNE_PITCH) {
            Serial.print("Tuning:PITCH cycles="); Serial.print(rt_pitch.valid_cycles);
            Serial.print("/"); Serial.print(AUTOTUNE_AVG_CYCLES);
        }
        Serial.println();
    }
}