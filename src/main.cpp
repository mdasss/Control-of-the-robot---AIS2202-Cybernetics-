#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"
#include <thread>
#include <chrono>

AS5600 ASL;
AS5600 ASL_shoulder;
TwoWire Wire_shoulder = TwoWire(1);

// Pin definitions wrist
const int Single_channel_pin = 26; // STBY / enable
const int InA_pin = 33;
const int InB_pin = 25;
const int PWM_pin = 32;

const int SCL_pin = 23;
const int SDA_pin = 22;

// Shoulder motor
const int Single_channel_pin_sh = 14;
const int InA_pin_sh = 27;
const int InB_pin_sh = 12;
const int PWM_pin_sh = 13;

const int SCL_pin_sh = 19;
const int SDA_pin_sh = 18;

// PID
float Target_Deg = 0;
float target_deg_sh = 8;
float Kp = 3, Ki = 0.2, Kd = 0.4;

// Motor limits
const int PWM_max = 170;
const int PWM_min = 60;
const float tolerance = 1;
const uint32_t DT_ms = 10;

// PID states
float err_prev = 0, err_int = 0;
float err_prev_sh = 0, err_int_sh = 0;

int cmd_prev = 0, cmd_prev_sh = 0;
const int PWM_slew = 5;

unsigned long lastPrint = 0;
const unsigned long printInterval = 200;

// Encoder wrap state
int turns_w = 0, turns_sh = 0;
uint16_t prev_raw_w = 0, prev_raw_sh = 0;
constexpr float RAW2DEG = 360.0f / 4096.0f;

// Gear ratios
const float GEAR_W = 4.0f;
const float GEAR_SH = 4.0f;

void setup() {
    Serial.begin(115200);
    delay(100);

    // I2C setup
    Wire.begin(SDA_pin, SCL_pin);
    Wire_shoulder.begin(SDA_pin_sh, SCL_pin_sh);

    // Initialize encoders
    ASL.setWire(&Wire);
    ASL.begin(-1);
    ASL.setDirection(AS5600_CLOCK_WISE);

    ASL_shoulder.setWire(&Wire_shoulder);
    ASL_shoulder.begin(-1);
    ASL_shoulder.setDirection(AS5600_CLOCK_WISE);

    // Motor pins
    pinMode(PWM_pin, OUTPUT);
    pinMode(PWM_pin_sh, OUTPUT);
    pinMode(Single_channel_pin, OUTPUT);
    pinMode(Single_channel_pin_sh, OUTPUT);
    pinMode(InA_pin, OUTPUT);
    pinMode(InB_pin, OUTPUT);
    pinMode(InA_pin_sh, OUTPUT);
    pinMode(InB_pin_sh, OUTPUT);

    // ---- SAFE STBY STARTUP ----
    // Hold motors disabled by default
    digitalWrite(Single_channel_pin, LOW);
    digitalWrite(Single_channel_pin_sh, LOW);
    delay(100); // small delay to let driver fully disable
    // Enable control
    digitalWrite(Single_channel_pin, HIGH);
    digitalWrite(Single_channel_pin_sh, HIGH);
    // ---------------------------

    analogWrite(PWM_pin, 0);
    analogWrite(PWM_pin_sh, 0);

    // Check sensor connections
    if (!ASL.isConnected()) {
        Serial.println("Wrist encoder not found!");
        while (1);
    }
    if (!ASL_shoulder.isConnected()) {
        Serial.println("Shoulder encoder not found!");
        while (1);
    }

    Serial.println("Encoders found.");

    prev_raw_w = ASL.rawAngle();
    prev_raw_sh = ASL_shoulder.rawAngle();
}

void loop() {
    float raw_deg = ASL.rawAngle();
    float raw_deg_sh = ASL_shoulder.rawAngle();
    unsigned long currentMillis = millis();

    // Wrap detection
    int dw = (int)raw_deg - (int)prev_raw_w;
    if (dw < -2048) turns_w++;
    else if (dw > 2048) turns_w--;
    prev_raw_w = (uint16_t)raw_deg;

    int dsh = (int)raw_deg_sh - (int)prev_raw_sh;
    if (dsh < -2048) turns_sh++;
    else if (dsh > 2048) turns_sh--;
    prev_raw_sh = (uint16_t)raw_deg_sh;

    float angle_w_deg = (turns_w * 4096 + (int)raw_deg) * RAW2DEG;
    float angle_sh_deg = (turns_sh * 4096 + (int)raw_deg_sh) * RAW2DEG;

    auto mod360 = [](float a) {
        a = fmodf(a, 360.0f);
        if (a < 0) a += 360.0f;
        return a;
    };
    float meas_w_mod = mod360(angle_w_deg / GEAR_W);
    float meas_sh_mod = mod360(angle_sh_deg / GEAR_SH);

    auto wrapTo180 = [](float x) {
        while (x > 180.0f) x -= 360.0f;
        while (x <= -180.0f) x += 360.0f;
        return x;
    };

    if (Serial.available()) {
        String s = Serial.readStringUntil('\n');
        s.trim();
        if (s.length()) {
            if (s.startsWith("w")) {
                Target_Deg = s.substring(1).toFloat();
                while (Target_Deg >= 360.0) Target_Deg -= 360.0;
                while (Target_Deg < 0.0) Target_Deg += 360.0;
                err_int = err_prev = 0;
                Serial.print("New wrist target: ");
                Serial.println(Target_Deg, 2);
            } else if (s.startsWith("s")) {
                target_deg_sh = s.substring(1).toFloat();
                while (target_deg_sh >= 360.0) target_deg_sh -= 360.0;
                while (target_deg_sh < 0.0) target_deg_sh += 360.0;
                err_int_sh = err_prev_sh = 0;
                Serial.print("New shoulder target: ");
                Serial.println(target_deg_sh, 2);
            } else {
                Serial.println("Invalid input. Use w### or s###");
            }
        }
    }

    static uint32_t t0 = 0;
    uint32_t now = millis();
    if (now - t0 < DT_ms) return;
    float dt = (now - t0) / 1000.0;
    t0 = now;

    float e = wrapTo180(Target_Deg - meas_w_mod);
    float e_sh = wrapTo180(target_deg_sh - meas_sh_mod);

    if (fabs(e) <= tolerance) {
        err_int = 0;
        err_prev = e;
        analogWrite(PWM_pin, 0);
        digitalWrite(InA_pin, LOW);
        digitalWrite(InB_pin, LOW);
    }
    if (fabs(e_sh) <= tolerance) {
        err_int_sh = 0;
        err_prev_sh = e_sh;
        analogWrite(PWM_pin_sh, 0);
        digitalWrite(InA_pin_sh, LOW);
        digitalWrite(InB_pin_sh, LOW);
    }

    err_int += e * dt;
    err_int_sh += e_sh * dt;

    if (Ki > 0.0001f) {
        float i_max = (float)PWM_max / Ki;
        err_int = constrain(err_int, -i_max, i_max);
        err_int_sh = constrain(err_int_sh, -i_max, i_max);
    }

    float de = (e - err_prev) / dt;
    float de_sh = (e_sh - err_prev_sh) / dt;
    err_prev = e;
    err_prev_sh = e_sh;

    float u = Kp * e + Ki * err_int + Kd * de;
    float u_sh = Kp * e_sh + Ki * err_int_sh + Kd * de_sh;

    int cmd = (int)round(u);
    int cmd_sh = (int)round(u_sh);

    cmd = constrain(cmd, -PWM_max, PWM_max);
    cmd_sh = constrain(cmd_sh, -PWM_max, PWM_max);

    if (abs(cmd) < PWM_min && fabs(e) > tolerance)
        cmd = (cmd >= 0) ? PWM_min : -PWM_min;
    if (abs(cmd_sh) < PWM_min && fabs(e_sh) > tolerance)
        cmd_sh = (cmd_sh >= 0) ? PWM_min : -PWM_min;

    int delta = cmd - cmd_prev;
    int delta_sh = cmd_sh - cmd_prev_sh;
    if (delta > PWM_slew) cmd = cmd_prev + PWM_slew;
    else if (delta < -PWM_slew) cmd = cmd_prev - PWM_slew;
    cmd_prev = cmd;

    if (delta_sh > PWM_slew) cmd_sh = cmd_prev_sh + PWM_slew;
    else if (delta_sh < -PWM_slew) cmd_sh = cmd_prev_sh - PWM_slew;
    cmd_prev_sh = cmd_sh;

    if (fabs(e) <= tolerance) { cmd = 0; err_int = 0; }
    if (fabs(e_sh) <= tolerance) { cmd_sh = 0; err_int_sh = 0; }

    // >>> SAFE STBY handling <<<
    bool driving = (cmd != 0) || (cmd_sh != 0);
    digitalWrite(Single_channel_pin, driving ? HIGH : LOW);
    digitalWrite(Single_channel_pin_sh, driving ? HIGH : LOW);
    // >>>>>>>>>>>>>>>>>>>>>>>>>>

    if (currentMillis - lastPrint >= printInterval) {
        lastPrint = currentMillis;
        Serial.print("Wrist raw: "); Serial.println(raw_deg);
        Serial.print("Shoulder raw: "); Serial.println(raw_deg_sh);
        Serial.print("W out: "); Serial.println(meas_w_mod);
        Serial.print("S out: "); Serial.println(meas_sh_mod);
        Serial.print("Cmd W: "); Serial.println(cmd);
        Serial.print("Cmd S: "); Serial.println(cmd_sh);
    }

    // Direction control
    if (cmd > 0) {
        digitalWrite(InA_pin, HIGH);
        digitalWrite(InB_pin, LOW);
        analogWrite(PWM_pin, cmd);
    } else if (cmd < 0) {
        digitalWrite(InA_pin, LOW);
        digitalWrite(InB_pin, HIGH);
        analogWrite(PWM_pin, -cmd);
    } else {
        analogWrite(PWM_pin, 0);
        digitalWrite(InA_pin, LOW);
        digitalWrite(InB_pin, LOW);
    }

    if (cmd_sh > 0) {
        digitalWrite(InA_pin_sh, HIGH);
        digitalWrite(InB_pin_sh, LOW);
        analogWrite(PWM_pin_sh, cmd_sh);
    } else if (cmd_sh < 0) {
        digitalWrite(InA_pin_sh, LOW);
        digitalWrite(InB_pin_sh, HIGH);
        analogWrite(PWM_pin_sh, -cmd_sh);
    } else {
        analogWrite(PWM_pin_sh, 0);
        digitalWrite(InA_pin_sh, LOW);
        digitalWrite(InB_pin_sh, LOW);
    }
}


// the robot is moving after it loses connection with arduino, why? if the angle is equal target deg!