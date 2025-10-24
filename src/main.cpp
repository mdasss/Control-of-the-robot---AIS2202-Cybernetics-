#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"
#include <thread>   // For std::this_thread::sleep_for
#include <chrono>

AS5600 ASL;  // Create sensor/encoder object
AS5600 ASL_shoulder;
TwoWire Wire_shoulder = TwoWire(1); // second bus for the shoulder encoder

// Pin definitions wrist
const int Single_channel_pin = 26; // STBY / Single-channel enable, 21 before
const int InA_pin = 33;  // INA1
const int InB_pin = 25;  // INA2
const int PWM_pin = 32;  // PWMA

const int SCL_pin = 23;  // I2C SCL
const int SDA_pin = 22;  // I2C SDA

// PWM configuration

const int Single_channel_pin_sh = 14; // might be necessary to change alle of the motor control pins
const int InA_pin_sh = 27;
const int InB_pin_sh = 12;
const int PWM_pin_sh = 13;

const int SCL_pin_sh = 19;
const int SDA_pin_sh = 18;

// PID and target
float  Target_Deg = 0; // change! overwrites the actual position after the serial monitor has been turned off
float target_deg_sh = 8; // this one too
float Kp = 3, Ki = 0.2, Kd = 0.4; // might be necessary to use some other values for the constants of the second PID

// motor limits, important! without that, the motor driver cuts off, if motor spins too fast
const int PWM_max = 170;
const int PWM_min = 60;
const float tolerance = 1; // 1.5 was best
const uint32_t DT_ms = 10; // difference in time in ms, control frequency

// pid states
float err_prev = 0;
float err_int = 0;

float err_prev_sh = 0;
float err_int_sh = 0;

int cmd_prev = 0;          // last PWM command
int cmd_prev_sh = 0;
const int PWM_slew = 5;  // max change per control loop,  tune 3–10., 5 worked best so
bool motorsPaused = false;

unsigned long lastPrint = 0;
const unsigned long printInterval = 200; // max pause time for printing of the angles

// ---- NEW wrap system state ----
int turns_w = 0, turns_sh = 0;                 // multi-turn counters
uint16_t prev_raw_w = 0, prev_raw_sh = 0;      // previous raw samples (0..4095)
constexpr float RAW2DEG = 360.0f/4096.0f;      // raw counts -> degrees

// ---- Gear ratios (sensor/motor turns per 1 output turn) ----
const float GEAR_W  = 4.0f;   // wrist (set to your actual ratio)
const float GEAR_SH = 4.0f;   // shoulder (change if different)

// -------------------------------

void setup() {
    Serial.begin(115200);
    delay(100);

    // I2C setup,  sensor comm
    Wire.begin(SDA_pin, SCL_pin);
    Wire_shoulder.begin(SDA_pin_sh, SCL_pin_sh);

    // Initialize AS5600
    ASL.setWire(&Wire);
    ASL.begin(-1); // No direction pin
    ASL.setDirection(AS5600_CLOCK_WISE); // increasing angle configfured as clock wise

    ASL_shoulder.setWire(&Wire_shoulder);
    ASL_shoulder.begin(-1);
    ASL_shoulder.setDirection(AS5600_CLOCK_WISE);

    // Configure motor driver pins
    pinMode(PWM_pin, OUTPUT);
    pinMode(PWM_pin_sh, OUTPUT);

    pinMode(Single_channel_pin, OUTPUT);
    pinMode(Single_channel_pin_sh, OUTPUT);

    pinMode(InA_pin, OUTPUT);
    pinMode(InB_pin, OUTPUT);

    pinMode(InA_pin_sh, OUTPUT);
    pinMode(InB_pin_sh, OUTPUT);

    // did not work
    //pinMode(Fault_pin1, INPUT_PULLUP);
    //pinMode(Fault_pin2, INPUT_PULLUP);

    // Enable single-channel mode / STBY
    digitalWrite(Single_channel_pin, HIGH);
    digitalWrite(Single_channel_pin_sh, HIGH);

    analogWrite(PWM_pin,0);
    analogWrite(PWM_pin_sh, 0);

    // Check sensor connection
    if (!ASL.isConnected()) {
        Serial.println("Wrist encoder has not been found, check the wires!!");
        while (1); // halt
    }
    if (!ASL_shoulder.isConnected()){
        Serial.println("Shoulder encoder has not been found, check the wires");
        while(1);
    }
    // not that important
    Serial.println("Found the encoders");
    Serial.print("I2C Address: 0x");
    Serial.println(ASL.getAddress(), HEX);
    Serial.print("I2C Address: 0x");
    Serial.println(ASL_shoulder.getAddress(), HEX);

    prev_raw_w  = ASL.rawAngle();
    prev_raw_sh = ASL_shoulder.rawAngle();

    // normalize the target to be within [0,360) degrees //
    while (Target_Deg >= 360.0) Target_Deg -= 360.0;
    while (Target_Deg < 0.0) Target_Deg += 360.0;

    while (target_deg_sh >= 360.0) target_deg_sh -= 360.0;
    while (target_deg_sh < 0.0) target_deg_sh += 360.0;

}

void loop() {
    // Forward direction, for testing
    // digitalWrite(InA_pin, HIGH);
    //digitalWrite(InB_pin, LOW);

    // Read AS5600 angle
    //float angle_deg = ASL.readAngle(); shittt
    float raw_deg = ASL.rawAngle(); //* AS5600_RAW_TO_DEGREES; // just used for debuging
    float raw_deg_sh = ASL_shoulder.rawAngle();
    unsigned long currentMillis = millis();

    // ----- NEW wrap/unwrap -----
    // update multi-turn counters using wrap detection (half-scale threshold = 2048)
    int dw = (int)raw_deg - (int)prev_raw_w;
    if (dw < -2048)      turns_w++;   // wrapped forward (359->0)
    else if (dw > 2048)  turns_w--;   // wrapped backward (0->359)
    prev_raw_w = (uint16_t)raw_deg;

    int dsh = (int)raw_deg_sh - (int)prev_raw_sh;
    if (dsh < -2048)      turns_sh++;
    else if (dsh > 2048)  turns_sh--;
    prev_raw_sh = (uint16_t)raw_deg_sh;

    // absolute unwrapped angles in degrees (sensor space)
    float angle_w_deg  = (turns_w  * 4096 + (int)raw_deg)    * RAW2DEG;
    float angle_sh_deg = (turns_sh * 4096 + (int)raw_deg_sh) * RAW2DEG;

    // reduce measured angles to [0..360) in **output space** by dividing by gear ratio
    auto mod360 = [](float a){
      a = fmodf(a, 360.0f);
      if (a < 0) a += 360.0f;
      return a;
    };
    float meas_w_mod  = mod360(angle_w_deg  / GEAR_W);   // << scale feedback
    float meas_sh_mod = mod360(angle_sh_deg / GEAR_SH);  // << scale feedback

    // shortest-path helper (−180..+180)
    auto wrapTo180 = [](float x){
      while (x > 180.0f)   x -= 360.0f;
      while (x <= -180.0f) x += 360.0f;
      return x;
    };
    // ---------------------------

    /*
    if (raw_deg_sh < prev_deg_sh - 2048) n_sh++;
    else if (raw_deg_sh > prev_deg_sh + 2048) n_sh--;

    prev_deg_sh = raw_deg_sh;


    raw_deg += n_w * 4096;
    if (raw_deg < prev_deg_w) {
        n_w++;
        if (n_w == 4) n_w = 0; prev_deg_w = 0;
    }
    else if (raw_deg > prev_deg_w) {
        n_w--;
        if (n_w == -4) n_w = 0; prev_deg_w = 0;
    }
    prev_deg_w = raw_deg;
*/


    /*
    // bbbb: if paused, just keep motors off and skip control
    if (motorsPaused) {
        analogWrite(PWM_pin, 0);
        analogWrite(PWM_pin_sh, 0);
        digitalWrite(InA_pin, LOW);
        digitalWrite(InB_pin, LOW);
        digitalWrite(InA_pin_sh, LOW);
        digitalWrite(InB_pin_sh, LOW);
        delay(50);
        return;
    }
     */

    /*
    Serial.print("\tRaw for the wrist: ");
    Serial.println(raw_deg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    Serial.print("\tRaw for the shoulder: ");
    Serial.println(raw_deg_sh);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
*/
    // chatgpt //
    // change target while the motor is rotating
    // "w120" move wrist to 120 degrees, or "s60" to move shoulder to 60 degrees
    if (Serial.available()) {
        String s = Serial.readStringUntil('\n');
        s.trim();  // remove spaces/newlines

        if (s.length()){ // ignore empty input

            if (s.startsWith("w")) {  // update to take into account the new values,
                Target_Deg = s.substring(1).toFloat(); // extract the numeric part of the input, 120, 60 and makes into 120.0, 60.0 etc..
                while (Target_Deg >= 360.0) Target_Deg -= 360.0; // normalize to output to stay within 0 and 360 degrees
                while (Target_Deg < 0.0) Target_Deg += 360.0;
                err_int = 0; err_prev = 0; // reset PID
                Serial.print("New wrist target: ");
                Serial.println(Target_Deg, 2);
            }
            else if (s.startsWith("s")) {
                target_deg_sh = s.substring(1).toFloat();
                while (target_deg_sh >= 360.0) target_deg_sh -= 360.0;
                while (target_deg_sh < 0.0) target_deg_sh += 360.0;
                err_int_sh = 0; err_prev_sh = 0;
                Serial.print("New shoulder target: ");
                Serial.println(target_deg_sh, 2);
            }

            else {
                Serial.println("Invalid input. Use w### or s###");
            }
        }
    }

    // control logic
    static uint32_t t0 = 0; // keeps the value between the calls
    uint32_t now = millis();
    if (now - t0 < DT_ms) return; // skips the cont loop if less then 10 ms has passed,
    float dt = (now - t0) / 1000.0;  // seconds
    t0 = now;

    // --- error in OUTPUT space (targets are already in output degrees) ---
    // float angle = (raw_deg + 4096*n_w) *  AS5600_RAW_TO_DEGREES; //reads new values from sensor
    // float angle_sh =(raw_deg_sh + 4096*n_sh) * AS5600_RAW_TO_DEGREES;

    // the shortst error  [-180..+180]
    // float e = Target_Deg - angle; // error between the target angle and the current
    // float e_sh = target_deg_sh - angle_sh;

    float e    = wrapTo180(Target_Deg    - meas_w_mod);
    float e_sh = wrapTo180(target_deg_sh - meas_sh_mod);

    /*
    if (e > 180.0)  e -= 360.0; // wraps the error by mapping it into -180 to + 180 degrees, to find the shortest path to the target value
    if (e < -180.0) e += 360.0;

    if (e_sh > 180.0)  e_sh -= 360.0;
    if (e_sh < -180.0) e_sh += 360.0;

    */
    // Stop if within the tolerance area

    if (fabs(e) <= tolerance) { // fabs(e) = abs value of float e
        err_int = 0.0;
        err_prev = e; // current error
        analogWrite(PWM_pin, 0);
        digitalWrite(InA_pin, LOW);
        digitalWrite(InB_pin, LOW);
    }

    if (fabs(e_sh) <= tolerance) { // fabs(e) = abs value of float e
        err_int_sh = 0.0;
        err_prev_sh = e_sh; // current error
        analogWrite(PWM_pin_sh, 0);
        digitalWrite(InA_pin_sh, LOW);
        digitalWrite(InB_pin_sh, LOW);

    }

    // PID logic
    err_int += e * dt; // Ki * err_int, accumulates the error
    err_int_sh += e_sh * dt;

    // limit the integral, dont know if really necessary here, if ki is const
    if (Ki > 0.0001f) {
        float i_max = (float)PWM_max / Ki;
        if (err_int >  i_max) err_int =  i_max; // makes sure the integral, will not become higher or lower then the abs(max) value
        if (err_int < -i_max) err_int = -i_max;
        if (err_int_sh > i_max) err_int_sh = i_max;
        if (err_int_sh < -i_max) err_int_sh = -i_max;
    }

    float de = (e - err_prev) / dt; // rate of change (derivative)
    float de_sh = (e_sh - err_prev_sh) / dt;

    err_prev = e;
    err_prev_sh = e_sh;

    float u = Kp*e + Ki*err_int + Kd*de;       // pos one direction, neg the other
    float u_sh = Kp * e_sh + Ki *err_int_sh + Kd*de_sh;

    int cmd = (int)round(u); // int pwm command from the continous u - output
    int cmd_sh = (int)round(u_sh);

    if (cmd >  PWM_max) cmd =  PWM_max; // reduces the cmd ti be within -pwm_max to pwm_max
    if (cmd < -PWM_max) cmd = -PWM_max;
    if (abs(cmd) < PWM_min){
        cmd = (cmd >= 0) ? PWM_min : -PWM_min; } // cmd bigger then 0 => true -> cmd = PWM_min, cmd smaller then 0, cmd = -PWM_min

    if (cmd_sh >  PWM_max) cmd_sh =  PWM_max; // reduces the cmd ti be within -pwm_max to pwm_max
    if (cmd_sh < -PWM_max) cmd_sh = -PWM_max;
    if (abs(cmd_sh) < PWM_min){
        cmd_sh = (cmd_sh >= 0) ? PWM_min : -PWM_min; }

    // Slew-limit the PWM to avoid big jumps, quite important. Too big jump, and the shitty motor driver cuts the current
    int delta = cmd - cmd_prev; // change in the pwm command sent to the motor driver
    int delta_sh = cmd_sh - cmd_prev_sh;

    if (delta > PWM_slew) cmd = cmd_prev + PWM_slew;
    else if (delta < -PWM_slew) cmd = cmd_prev - PWM_slew;
    cmd_prev = cmd;

    if (delta_sh > PWM_slew) cmd_sh = cmd_prev_sh + PWM_slew;
    else if (delta_sh < -PWM_slew) cmd_sh = cmd_prev_sh - PWM_slew;

    cmd_prev_sh = cmd_sh;

    // bbbb: force zero per-axis when within tolerance so only the axis with error moves
    // is it still important here? Should i move it upwards? to the previous if (fabs(e)) ?
    if (fabs(e) <= tolerance) { cmd = 0; err_int = 0; }
    if (fabs(e_sh) <= tolerance) { cmd_sh = 0; err_int_sh = 0; }

    /*
    // bbbb: only apply PWM_min when error is *outside* tolerance; otherwise force 0 so it can settle
    if (fabs(e) > tolerance) {
        if (abs(cmd) < PWM_min) {
            cmd = (cmd >= 0) ? PWM_min : -PWM_min;
        }
    } else {
        cmd = 0;
    }

    // bbbb: same logic for shoulder so it can actually stop inside the band
    if (fabs(e_sh) > tolerance) {
        if (abs(cmd_sh) < PWM_min) {
            cmd_sh = (cmd_sh >= 0) ? PWM_min : -PWM_min;
        }
    } else {
        cmd_sh = 0;
    }
*/
    if (currentMillis - lastPrint >= printInterval) {
        lastPrint = currentMillis;
        Serial.print("\tRaw for the wrist: ");
        Serial.println(raw_deg);
        Serial.print("\tTurns_w: ");
        Serial.println(turns_w);
        Serial.print("\tRaw for the shoulder: ");
        Serial.println(raw_deg_sh);
        Serial.print("\tTurns_sh: ");
        Serial.println(turns_sh);
        Serial.print("\tW meas(out): ");
        Serial.println(meas_w_mod);
        Serial.print("\tS meas(out): ");
        Serial.println(meas_sh_mod);
        Serial.println(cmd);
        Serial.println(cmd_sh);
    }

    // Direction
    if (cmd > 0) {                  // Forward
        digitalWrite(InA_pin, HIGH);
        digitalWrite(InB_pin, LOW);
        analogWrite(PWM_pin, cmd);
    } else {                        // Reverse
        digitalWrite(InA_pin, LOW);
        digitalWrite(InB_pin, HIGH);
        analogWrite(PWM_pin, -cmd);
    }
    if (cmd_sh > 0) {                  // Forward
        digitalWrite(InA_pin_sh, HIGH);
        digitalWrite(InB_pin_sh, LOW);
        analogWrite(PWM_pin_sh, cmd_sh);
    } else {                        // Reverse
        digitalWrite(InA_pin_sh, LOW);
        digitalWrite(InB_pin_sh, HIGH);
        analogWrite(PWM_pin_sh, -cmd_sh);
    }
}

// the robot is moving after it loses connection with arduino, why? if the angle is equal target deg!