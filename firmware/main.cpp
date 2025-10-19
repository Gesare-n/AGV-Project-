#include <Arduino.h> 

const unsigned long SERIAL_BAUD_RATE = 115200;
const unsigned long PID_UPDATE_INTERVAL_MS = 5; 
const unsigned long RPM_CALC_INTERVAL_MS = 4; 
const unsigned long SERIAL_TIMEOUT_MS = 1000; // Timeout for serial commands 
const int SERIAL_BUFFER_SIZE = 100; // Max size for incoming serial commands


int buzzer = PB13;
int buzzerTime;
float prev_cmd_time, cmd_timout = 100;

char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
bool commandReady = false;

unsigned long lastPidUpdateTime = 0;
unsigned long lastRpmCalcTime = 0;


typedef struct {
  // Encoder pins
  uint8_t ENCODER_A;
  uint8_t ENCODER_B;

  // Motor control pins
  uint8_t DIR1;
  uint8_t DIR2;
  uint8_t SpeedPin;
  uint8_t PWM_Channel; 

  // Encoder parameters
  float PULSES_PER_REV; // Encoder PPR 
  float COUNTS_PER_REV; // Actual counts per rev 

  // PID parameters
  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  float integral = 0.0;
  float previous_error = 0.0;
  float PID_scaling_factor = 0.0;

  
  volatile long encoder_count = 0; 
  volatile unsigned long last_pulse_time_us = 0;
  volatile int last_A_state; 
  volatile int last_B_state; 

  long prev_encoder_count = 0; // For RPM calculation
  unsigned long prev_rpm_calc_time_us = 0;

  int current_rpm = 0;
  int target_rpm = 0;
  int pwm_value = 0;
  bool direction_actual = true; // Actual direction based on encoder reading 
  bool is_quadrature;

} encoded_motor_info;

// Create global instances of the struct
encoded_motor_info frontRight_motor, frontLeft_motor, backRight_motor, backLeft_motor;


// Generic ISR handler template
void handleQuadratureEncoderInterrupt(encoded_motor_info* pMotor) {
    int a_state = digitalRead(pMotor->ENCODER_A);
    int b_state = digitalRead(pMotor->ENCODER_B);
    pMotor->last_pulse_time_us = micros(); // Record time of last activity


    if (1/a_state != pMotor->last_A_state/) { // Check if A changed
         if (a_state == HIGH) { // Rising edge on A
             if (b_state == LOW) {
                 pMotor->encoder_count++; // Forward
                 pMotor->direction_actual = true;
             } else {
                 pMotor->encoder_count--; // Reverse
                 pMotor->direction_actual = false;
             }
         }
         
    }
    

    pMotor->last_A_state = a_state; // Store current state for next time
    pMotor->last_B_state = b_state;
}

// Generic ISR handler for Single-Channel Encoders
void handleSingleChannelEncoderInterrupt(encoded_motor_info* pMotor) {
    // Only need to increment count and update time
    pMotor->encoder_count++; // Always increment for single channel
    pMotor->last_pulse_time_us = micros();
    // No direction check possible here
}


void frontRight_ISR() { handleQuadratureEncoderInterrupt(&frontRight_motor); }
void frontLeft_ISR()  { handleQuadratureEncoderInterrupt(&frontLeft_motor); }
void backRight_ISR()  { handleQuadratureEncoderInterrupt(&backRight_motor); }
void backLeft_ISR()   { handleQuadratureEncoderInterrupt(&backLeft_motor); }


void initMotors() {
    backLeft_motor.DIR1 = PA5;
    backLeft_motor.DIR2 = PA4;
    backLeft_motor.SpeedPin = PA0;
    backLeft_motor.ENCODER_A = PB0;
    backLeft_motor.ENCODER_B = PB1; 

    backRight_motor.DIR1 = PB3;
    backRight_motor.DIR2 = PB4;
    backRight_motor.SpeedPin = PB8;
    backRight_motor.ENCODER_B = PA11;
    backRight_motor.ENCODER_A = PA8; 

    frontLeft_motor.DIR1 = PA3;
    frontLeft_motor.DIR2 = PA2;
    frontLeft_motor.SpeedPin = PA1;
    frontLeft_motor.ENCODER_A = PA7;
    frontLeft_motor.ENCODER_B = PA6; 

    frontRight_motor.DIR1 = PB5;
    frontRight_motor.DIR2 = PB6;
    frontRight_motor.SpeedPin = PB7;
    frontRight_motor.ENCODER_A = PA12;
    frontRight_motor.ENCODER_B = PA15;

    backRight_motor.PULSES_PER_REV = 720.2; // ORIGINALLY 718
    backLeft_motor.PULSES_PER_REV = 720.2;  // 
    frontRight_motor.PULSES_PER_REV = 1978.3;
    frontLeft_motor.PULSES_PER_REV = 1978.3;

    // Back Motors (Quadrature)
    backLeft_motor.is_quadrature = true;
    backRight_motor.is_quadrature = true;

    backLeft_motor.COUNTS_PER_REV = backLeft_motor.PULSES_PER_REV * 1;
    backRight_motor.COUNTS_PER_REV = backRight_motor.PULSES_PER_REV * 1;

    frontLeft_motor.is_quadrature = true;
    frontRight_motor.is_quadrature = true;

    frontLeft_motor.COUNTS_PER_REV = frontLeft_motor.PULSES_PER_REV;
    frontRight_motor.COUNTS_PER_REV = frontRight_motor.PULSES_PER_REV;

    // --- PID Tuning --- 
    backRight_motor.kp = 5.0; backRight_motor.ki = 75.0; backRight_motor.kd = 200;//5.0; // KD was very high? 300//p300i10d30
    backLeft_motor.kp = 5.0; backLeft_motor.ki = 75.0; backLeft_motor.kd = 200;//5.0;
    backLeft_motor.PID_scaling_factor = backRight_motor.PID_scaling_factor = 100.0;

    frontLeft_motor.kp = 10.0; frontLeft_motor.ki = 85.0; frontLeft_motor.kd = 100;//1.0; // KD was 10
    frontRight_motor.kp = 10.0; frontRight_motor.ki = 85.0; frontRight_motor.kd = 100;//1.0; // KD was 10
    frontLeft_motor.PID_scaling_factor = frontRight_motor.PID_scaling_factor = 100.0;

    // --- Pin Modes & Interrupts ---
    encoded_motor_info* motors[] = {&frontRight_motor, &frontLeft_motor, &backRight_motor, &backLeft_motor};
    // ISRs 
    void (*front_isrs[])() = {frontRight_ISR, frontLeft_ISR};
    void (*back_isrs[])() = {backRight_ISR, backLeft_ISR}; 

    pinMode(frontRight_motor.DIR1, OUTPUT);
    pinMode(frontRight_motor.DIR2, OUTPUT);
    pinMode(frontRight_motor.SpeedPin, OUTPUT);
    pinMode(frontRight_motor.ENCODER_A, INPUT_PULLUP);
    pinMode(frontRight_motor.ENCODER_B, INPUT_PULLUP);
    frontRight_motor.last_A_state = digitalRead(frontRight_motor.ENCODER_A);
    frontRight_motor.last_B_state = digitalRead(frontRight_motor.ENCODER_B);
    attachInterrupt(digitalPinToInterrupt(frontRight_motor.ENCODER_A), frontRight_ISR, RISING);

    pinMode(frontLeft_motor.DIR1, OUTPUT);
    pinMode(frontLeft_motor.DIR2, OUTPUT);
    pinMode(frontLeft_motor.SpeedPin, OUTPUT);
    pinMode(frontLeft_motor.ENCODER_A, INPUT_PULLUP);
    pinMode(frontLeft_motor.ENCODER_B, INPUT_PULLUP);
    frontLeft_motor.last_A_state = digitalRead(frontLeft_motor.ENCODER_A);
    frontLeft_motor.last_B_state = digitalRead(frontLeft_motor.ENCODER_B);
    attachInterrupt(digitalPinToInterrupt(frontLeft_motor.ENCODER_A), frontLeft_ISR, RISING);


    pinMode(backRight_motor.DIR1, OUTPUT);
    pinMode(backRight_motor.DIR2, OUTPUT);
    pinMode(backRight_motor.SpeedPin, OUTPUT);
    pinMode(backRight_motor.ENCODER_A, INPUT_PULLUP);
    pinMode(backRight_motor.ENCODER_B, INPUT_PULLUP);
    backRight_motor.last_A_state = digitalRead(backRight_motor.ENCODER_A);
    backRight_motor.last_B_state = digitalRead(backRight_motor.ENCODER_B);
    attachInterrupt(digitalPinToInterrupt(backRight_motor.ENCODER_A), backRight_ISR, RISING);

    pinMode(backLeft_motor.DIR1, OUTPUT);
    pinMode(backLeft_motor.DIR2, OUTPUT);
    pinMode(backLeft_motor.SpeedPin, OUTPUT);
    pinMode(backLeft_motor.ENCODER_A, INPUT_PULLUP);
    pinMode(backLeft_motor.ENCODER_B, INPUT_PULLUP);
    backLeft_motor.last_A_state = digitalRead(backLeft_motor.ENCODER_A);
    backLeft_motor.last_B_state = digitalRead(backLeft_motor.ENCODER_B);
    attachInterrupt(digitalPinToInterrupt(backLeft_motor.ENCODER_A), backLeft_ISR, RISING);  
}


void calculateRPM(encoded_motor_info* pMotor) {
    long current_count;
    float current_time_us;
    float delta_time_us;
    long delta_count;

    
    //noInterrupts();
    current_count = pMotor->encoder_count;
    current_time_us = millis();
    //interrupts();

    delta_time_us = current_time_us - pMotor->prev_rpm_calc_time_us;
    delta_count = current_count - pMotor->prev_encoder_count;

    // Handle micros() rollover
    // if (current_time_us < pMotor->prev_rpm_calc_time_us) {
    //     delta_time_us = (0xFFFFFFFF - pMotor->prev_rpm_calc_time_us) + current_time_us;
    // }

    if (delta_time_us > 0 && pMotor->COUNTS_PER_REV > 0) {

        pMotor->current_rpm = (((delta_count/pMotor->COUNTS_PER_REV)/delta_time_us)*60000)*2;//    (float)delta_count * 60000 / ((float)delta_time_us * pMotor->COUNTS_PER_REV);

        // For quadrature, delta_count can be negative, giving signed RPM.
        // However, PID often works better with magnitude, let's adjust in PID update.
        // If you need signed RPM for quadrature here, keep it as is.
        // If you want magnitude for all, use:
        // pMotor->current_rpm = fabs((float)delta_count * 60000000.0f / ((float)delta_time_us * pMotor->COUNTS_PER_REV));

    } else {
        // Check for timeout to set RPM to 0 if stopped
        //noInterrupts();
        unsigned long last_pulse = pMotor->last_pulse_time_us;
        //interrupts();
        
        if ((current_time_us - last_pulse) > 10) { 
             pMotor->current_rpm = 0.0;
        }

    }

    // Store current values for the next calculation
    pMotor->prev_encoder_count = current_count;
    pMotor->prev_rpm_calc_time_us = current_time_us;
}

void updatePID(encoded_motor_info* pMotor, float dt_seconds) {
    int target = pMotor->target_rpm;
    int current = pMotor->current_rpm; 
    int error;

    if (pMotor->is_quadrature) {
           error = abs(target) - abs(current); 
           
    } else {
        error = abs(target) - current; 
    }

    float pTerm = pMotor->kp * error;

    // Integral Term (with anti-windup)
    pMotor->integral += pMotor->ki * error;// * dt_seconds;
    float max_integral = 4095.0; 
    pMotor->integral = constrain(pMotor->integral, 0, max_integral);

    float derivative = 0.0;
    if (dt_seconds > 0.00001f) {
         derivative = (error - pMotor->previous_error);
    }
    float dTerm = pMotor->kd * derivative;
    float output = (pTerm + pMotor->integral + dTerm);

    pMotor->previous_error = error;

    bool target_direction_is_forward = (target >= 0);

    // PWM magnitude is the absolute value of the signed PID output
    int pwm_magnitude = output;
    pMotor->pwm_value = constrain(pwm_magnitude, 0, 4095);

    if (fabs(target) < 1.0) { // Small deadband around zero target speed
        analogWrite(pMotor->SpeedPin, 0);
        pMotor->pwm_value = 0;
        pMotor->integral = 0; 
    } else {
        digitalWrite(pMotor->DIR1, target_direction_is_forward);
        digitalWrite(pMotor->DIR2, !target_direction_is_forward);
        analogWrite(pMotor->SpeedPin, pMotor->pwm_value);
    }
}

// --- Serial Communication ---

void handleSerialInput() {
  while (Serial.available() > 0) {
      Serial.println("Serial data received");
      prev_cmd_time = millis();
      char incomingByte = Serial.read();

    if (incomingByte == '\n' || incomingByte == '\r') { 
      if (serialBufferIndex > 0) { 
        serialBuffer[serialBufferIndex] = '\0'; 
        commandReady = true;
      }
    } else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialBufferIndex++] = incomingByte;
    } else {
      serialBufferIndex = 0;
      Serial.println("ERR: Buffer Overflow"); 
    }
  }
}

void processCommand() {
  if (!commandReady) {
      return;
  }
  
  if (serialBuffer[0] == 'm') {
    int t1, t2, t3, t4;
    int itemsParsed = sscanf(serialBuffer, "m %d %d %d %d", &t1, &t2, &t3, &t4);
    
    // Serial.print("sscanf items parsed: ");
    // Serial.println(itemsParsed);
    
    if (itemsParsed == 4) {
      frontLeft_motor.target_rpm = t1;
      frontRight_motor.target_rpm = t2;
      backLeft_motor.target_rpm = t3;
      backRight_motor.target_rpm = t4;

      //Serial.print(t1); Serial.print(" "); Serial.print(t2); Serial.print(" "); Serial.print(t3); Serial.print(" "); Serial.print(t4); Serial.print(" ");
      Serial.println("OK"); // Acknowledge
    } else {
      Serial.println("ERR: Invalid 'm' format");
    }
  } else if (serialBuffer[0] == 'e' && serialBuffer[1] == '\0') { // Check it's just "e"
    sendEncoderCounts();
  } else if(serialBuffer[0] == 'b'){
    int t1;
    int itemsParsed = sscanf(serialBuffer, "b %d", &t1);
    Serial.println("OK");
    digitalWrite(buzzer, HIGH);
    delay(t1);
    digitalWrite(buzzer, LOW);
    delay(5);
    digitalWrite(buzzer, HIGH);
    delay(t1);
    digitalWrite(buzzer, LOW);
  }
  // handlers for other commands ('p', 'i', 'd' for tuning?)
  // else if (serialBuffer[0] == 'p') { ... }
  else {
    Serial.print("ERR: Unknown command: ");
    Serial.println(serialBuffer);
  }

  serialBufferIndex = 0;
  commandReady = false;
}

void sendEncoderCounts() {
    char txBuffer[100];
    long fl_count, fr_count, bl_count, br_count;

    noInterrupts();
    fl_count = frontLeft_motor.encoder_count;
    fr_count = frontRight_motor.encoder_count;
    bl_count = backLeft_motor.encoder_count;
    br_count = backRight_motor.encoder_count;
    interrupts();

    sprintf(txBuffer, "%ld %ld %ld %ld", fl_count, fr_count, bl_count, br_count);
    Serial.println(txBuffer);
}

void printMotorData() {
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) {
    lastPrintTime = millis();

    char buffer[200]; 
    sprintf(buffer, "F: Tgt:%d Rpm:%d Pwm:%d Cnt:%ld | Tgt:%d Rpm:%d Pwm:%d Cnt:%ld",
            frontLeft_motor.target_rpm, frontLeft_motor.current_rpm, frontLeft_motor.pwm_value, frontLeft_motor.encoder_count,
            frontRight_motor.target_rpm, frontRight_motor.current_rpm, frontRight_motor.pwm_value, frontRight_motor.encoder_count
    );
    Serial.print(buffer); 

    Serial.print("    ");

    sprintf(buffer, "B: Tgt:%d Rpm:%d Pwm:%d Cnt:%ld | Tgt:%d Rpm:%d Pwm:%d Cnt:%ld",
            backLeft_motor.target_rpm, backLeft_motor.current_rpm, backLeft_motor.pwm_value, backLeft_motor.encoder_count,
            backRight_motor.target_rpm, backRight_motor.current_rpm, backRight_motor.pwm_value, backRight_motor.encoder_count
    );
    Serial.println(buffer);
  }
}



void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial);

  pinMode(buzzer, OUTPUT);
  initMotors();

  unsigned long now = millis();
  lastPidUpdateTime = now;
  lastRpmCalcTime = now;
  unsigned long now_us = micros();
  frontLeft_motor.prev_rpm_calc_time_us = now_us;
  frontRight_motor.prev_rpm_calc_time_us = now_us;
  backLeft_motor.prev_rpm_calc_time_us = now_us;
  backRight_motor.prev_rpm_calc_time_us = now_us;

  frontLeft_motor.kp /= frontLeft_motor.PID_scaling_factor; 
  frontLeft_motor.ki /= frontLeft_motor.PID_scaling_factor; 
  frontLeft_motor.kd /= frontLeft_motor.PID_scaling_factor; 

  frontRight_motor.kp /= frontRight_motor.PID_scaling_factor; 
  frontRight_motor.ki /= frontRight_motor.PID_scaling_factor; 
  frontRight_motor.kd /= frontRight_motor.PID_scaling_factor; 

  backLeft_motor.kp /= backLeft_motor.PID_scaling_factor; 
  backLeft_motor.ki /= backLeft_motor.PID_scaling_factor; 
  backLeft_motor.kd /= backLeft_motor.PID_scaling_factor; 

  backRight_motor.kp /= backRight_motor.PID_scaling_factor; 
  backRight_motor.ki /= backRight_motor.PID_scaling_factor; 
  backRight_motor.kd /= backRight_motor.PID_scaling_factor; 

}

void loop() {

  // printMotorData();
  // frontLeft_motor.target_rpm = 00;
  // frontRight_motor.target_rpm = 00;
  // backLeft_motor.target_rpm = 00;
  // backRight_motor.target_rpm = 00;
  prev_cmd_time = millis();
      
  // while((millis() - prev_cmd_time) >= cmd_timout){
  //   handleSerialInput();
  //   backLeft_motor.target_rpm = 0;
  //   backRight_motor.target_rpm = 0;
  //   frontLeft_motor.target_rpm = 0;
  //   frontRight_motor.target_rpm = 0;
  //   float dt_sec = 1;
  //   updatePID(&frontLeft_motor, dt_sec);
  //   updatePID(&frontRight_motor, dt_sec);

  //   updatePID(&backLeft_motor, dt_sec);
  //   updatePID(&backRight_motor, dt_sec);
  // }


  handleSerialInput();
  processCommand(); 


  unsigned long currentMillis = millis();
  if (currentMillis - lastRpmCalcTime >= RPM_CALC_INTERVAL_MS) {
      lastRpmCalcTime = currentMillis; // Schedule next calc

      calculateRPM(&frontLeft_motor);
      calculateRPM(&frontRight_motor);
      calculateRPM(&backLeft_motor);
      calculateRPM(&backRight_motor);
  }

  if (currentMillis - lastPidUpdateTime >= PID_UPDATE_INTERVAL_MS) {
        float dt_sec = (currentMillis - lastPidUpdateTime) / 1000.0f; 
        lastPidUpdateTime = currentMillis; 

      updatePID(&frontLeft_motor, dt_sec);
      updatePID(&frontRight_motor, dt_sec);

      updatePID(&backLeft_motor, dt_sec);
      updatePID(&backRight_motor, dt_sec);
  }


  printMotorData();
  //serialPID();

}

void resetPID() {
  frontLeft_motor.previous_error = frontRight_motor.previous_error = 0;
  frontLeft_motor.integral = frontRight_motor.integral = 0;
  backLeft_motor.previous_error = backRight_motor.previous_error = 0;
  backLeft_motor.integral = backRight_motor.integral = 0;
}

void serialPID(){
if(Serial.available()){
  String data = Serial.readString();

  backRight_motor.kp = get_p(data);
  backRight_motor.ki = get_i(data);
  backRight_motor.kd = get_d(data);
  resetPID();
  //moving = 1;
}
}


float get_d(String data){
  data.remove(data.indexOf("p"),data.indexOf("d")+1);
  return data.toFloat(); 
}

float get_i(String data){
  data.remove(data.indexOf("p"),data.indexOf("i")+1);
  data.remove(data.indexOf("d"));
  return data.toFloat();
}

float get_p(String data){
  data.remove(data.indexOf("p"),1);
  data.remove(data.indexOf("i"));
  return data.toFloat();
}