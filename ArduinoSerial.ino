#include <TimerOne.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include "AS5600.h"

// Constants for the QMC5883L compass
const byte qmc5883l_mode_stby = 0x00;
const byte qmc5883l_mode_cont = 0x01;
const byte qmc5883l_odr_10hz  = 0x00;
const byte qmc5883l_odr_50hz  = 0x04;
const byte qmc5883l_odr_100hz = 0x08;
const byte qmc5883l_odr_200hz = 0x0C;
const byte qmc5883l_rng_2g    = 0x00;
const byte qmc5883l_rng_8g    = 0x10;
const byte qmc5883l_osr_512   = 0x00;
const byte qmc5883l_osr_256   = 0x40;
const byte qmc5883l_osr_128   = 0x80;
const byte qmc5883l_osr_64    = 0xC0;

QMC5883LCompass compass;
AS5600 as5600;

#define PCA9548A_ADDR 0x70
#define COMPASS_CHANNEL 0
#define AS5600_CHANNEL 1

// Pin definitions for the first motor driver
// Back Right
const int IN1 = 7;
const int IN2 = 9;
const int BR_PWM = 5;
const int BR_ENCODER_1 = 3; // interrupt pin for arduino mega
const int BR_ENCODER_2 = 11;

// Pin definitions for the second motor driver
// Back Left
const int IN3 = 28;
const int IN4 = 29;
const int BL_PWM = 4;
const int BL_ENCODER_1 = 2; // interrupt pin for arduino mega
const int BL_ENCODER_2 = 24;

// Pin definitions for the third motor driver
// Front Right
const int IN5 = 15;
const int IN6 = 14;
const int FR_PWM = 46;
const int FR_ENCODER_1 = 19; // interrupt pin for arduino mega
const int FR_ENCODER_2 = 17;

// Pin definitions for the fourth motor driver
// Front Left
const int IN7 = 13;
const int IN8 = 12;
const int FL_PWM = 45;
const int FL_ENCODER_1 = 18; // interrupt pin for arduino mega
const int FL_ENCODER_2 = 38;

double fl_rpm = 0, fr_rpm = 0;
double fr_w = 0, fl_w = 0;

double bl_rpm = 0, br_rpm = 0;
double br_w = 0, bl_w = 0;

// PID parameters
//volatile double targetrpm = 15; // from 8 to 35 rpm
double kpfr = 0.3;
double kifr = 0.3 *fr_w;
double kdfr = 0.0;

double kpfl = 0.3;
double kifl = 0.3 *fl_w;
double kdfl = 0.0;

double kpbr = 0.3;
double kibr = 0.3 *br_w;
double kdbr = 0.0;

double kpbl = 0.3;
double kibl = 0.3 *bl_w;
double kdbl = 0.0;

volatile float previousTime;

// Ticks Per Revolution
const double TPR = 12;

// One-second interval for measurements
int interval = 100;  // 10 Hz
long previousMillis = 0;
long currentMillis = 0;

double fl_error, fl_lastError, fl_input, bl_error, bl_lastError, bl_input;
double fl_cumError, fl_rateError, bl_cumError, bl_rateError;

double fr_error, fr_lastError, fr_input, br_error, br_lastError, br_input;
double fr_cumError, fr_rateError, br_cumError, br_rateError;

volatile double fr_encCount = 0;
volatile double fl_encCount = 0;
volatile double br_encCount = 0;
volatile double bl_encCount = 0;

double fl_out = 0;
double fr_out = 0;
double bl_out = 0;
double br_out = 0;

double speed_ang = 0, speed_lin = 0;

// Robot dimensions
double e = 0.375;  // Track width (m)
double r = 0.088;  // Wheel radius (m)
double x = 0.400;  // Wheel base (m)

float initialRawAngle = 0.0;
float normalizedAngle = 0.0;

void setup() {
  // Initialize motor driver pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(BR_PWM, OUTPUT);
  pinMode(BR_ENCODER_1, INPUT_PULLUP);
  pinMode(BR_ENCODER_2, INPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BL_ENCODER_1, INPUT_PULLUP);
  pinMode(BL_ENCODER_2, INPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(FR_ENCODER_1, INPUT_PULLUP);
  pinMode(FR_ENCODER_2, INPUT);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  pinMode(FL_PWM, OUTPUT);
  pinMode(FL_ENCODER_1, INPUT_PULLUP);
  pinMode(FL_ENCODER_2, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(FL_ENCODER_1), front_left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_ENCODER_1), front_right_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(BL_ENCODER_1), back_left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(BR_ENCODER_1), back_right_wheel_tick, RISING);

  Timer1.initialize(100000);
  Timer1.attachInterrupt(timerPID);

  // Initialize motor directions
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);

  Serial.begin(115200);

  // Initialize multiplexer
  Wire.begin();

  // Initialize AS5600
  switchMultiplexerChannel(AS5600_CHANNEL);
  as5600.begin(4);  // Set direction pin
  as5600.setDirection(AS5600_CLOCK_WISE);

  // Read initial raw angle and set it as zero reference
  initialRawAngle = as5600.rawAngle() * AS5600_RAW_TO_RADIANS;

  // Initialize QMC5883L compass
  switchMultiplexerChannel(COMPASS_CHANNEL);
  compass.init();
  compass.setCalibration(-2145, 422, -3700, -1057, -2187, -1207);
}

void loop() {
  delay(10);
  currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read encoder values and compute RPM
    float fl_revolutions = fl_encCount / TPR;
    float fr_revolutions = fr_encCount / TPR;
    float bl_revolutions = bl_encCount / TPR;
    float br_revolutions = br_encCount / TPR;
    fl_rpm = fl_revolutions * 600/103; //(600.0 / (interval * 1));  // interval is 100ms, so 10 intervals per second
    fr_rpm = fr_revolutions * 600/103; //(600.0 / (interval * 1));
    bl_rpm = bl_revolutions * 600/103;
    br_rpm = br_revolutions * 600/103;

    if (fl_out < 0) { fl_rpm = -fl_rpm; }
    if (fr_out < 0) { fr_rpm = -fr_rpm; }
    if (bl_out < 0) { bl_rpm = -bl_rpm; }
    if (br_out < 0) { br_rpm = -br_rpm; }

    fl_encCount = 0;
    fr_encCount = 0;
    bl_encCount = 0;
    br_encCount = 0;

    // Read current raw angle from AS5600 and subtract the initial angle to normalize
    switchMultiplexerChannel(AS5600_CHANNEL);
    float currentRawAngle = as5600.rawAngle() * AS5600_RAW_TO_RADIANS;
    normalizedAngle = currentRawAngle - initialRawAngle;

    // Wrap the normalized angle between -PI and PI
    normalizedAngle = fmod(normalizedAngle + PI, 2 * PI) - PI;

    // Read compass heading
    switchMultiplexerChannel(COMPASS_CHANNEL);
    compass.read();
    int azimuth = compass.getAzimuth();

    // Send data over Serial at 10 Hz
    //Serial.print("Heading: ");
    //Serial.print(azimuth);

    
    Serial.print("Angle: ");
    Serial.print(normalizedAngle);
    Serial.print(", FL RPM: ");
    Serial.print(fl_rpm);
    Serial.print(", FR RPM: ");
    Serial.print(fr_rpm);
    Serial.print(", BL RPM: ");
    Serial.print(bl_rpm);
    Serial.print(", BR RPM: ");
    Serial.println(br_rpm);

    
//    Serial.print(", Linear Speed: ");
//    Serial.print(speed_lin, 2);
//    Serial.print(", Angular Speed: ");
//    Serial.println(speed_ang, 2);
    //delay(10);
  }

  // Check for incoming serial data
  if (Serial.available()) {
      String data = Serial.readStringUntil('\n'); // Read a line of input
      int commaIndex = data.indexOf(',');
      if (commaIndex > 0) {
          String str_lin = data.substring(0, commaIndex);  // Extract linear velocity
          String str_ang = data.substring(commaIndex + 1); // Extract angular velocity
          speed_lin = str_lin.toFloat();  // Convert to float
          speed_ang = str_ang.toFloat();  // Convert to float
      }
      
  }

  
  // Calculate wheel velocities
  double A[4][2] = {
    {(x + e * tan(normalizedAngle) / 2) / (x * r), 0},
    {(x - e * tan(normalizedAngle) / 2) / (x * r), 0},
    {(x + e * sin(normalizedAngle) / 2) / (x * r * cos(normalizedAngle)), -e / (2 * r)},
    {(x - e * sin(normalizedAngle) / 2) / (x * r * cos(normalizedAngle)), e / (2 * r)}
  };

  fl_w = (A[0][0] * speed_lin + A[0][1] * speed_ang);
  fr_w = (A[1][0] * speed_lin + A[1][1] * speed_ang);
  bl_w = (A[2][0] * speed_lin + A[2][1] * speed_ang);
  br_w = (A[3][0] * speed_lin + A[3][1] * speed_ang);
//  Serial.print("br_w1 : ");
//  Serial.println(br_w);
}

void switchMultiplexerChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void timerPID() {

  double kpfr = 0.06* abs(fr_w);
  if (fl_w == 0) {kpfl = 0; kifl = 0;  kdfl = 0;}
  double kifr = 0.4 * abs(fr_w);
  double kdfr = 0.0;
  
  double kpfl = 0.06* abs(fl_w);
  if (br_w == 0) {kpbr = 0; kibr = 0;  kdbr = 0;}
  double kifl = 0.4 * abs(fl_w);
  double kdfl = 0.0;
  
  double kpbr = 0.06* abs(br_w);
  if (br_w == 0) {kpbr = 0; kibr = 0;  kdbr = 0;}
  double kibr = 0.4 * abs(br_w);
  double kdbr = 0.0;
  
  double kpbl = 0.06* abs(bl_w);
  if (bl_w == 0) {kpbl = 0; kibl = 0;  kdbl = 0;}
  double kibl = 0.4 * abs(bl_w);
  double kdbl = 0.0;

  long currentTime = micros();
  float deltaT = ((currentTime - previousTime)) / 1.0e6;
  previousTime = currentTime;

  fl_error = (fl_w) - fl_rpm;
  fl_cumError += fl_error * deltaT;
  fl_rateError = (fl_error - fl_lastError) / deltaT;
  fl_out = kpfl * fl_error + kifl * fl_cumError + kdfl * fl_rateError;

  analogWrite(FL_PWM, abs(fl_out));
  fl_lastError = fl_error;
  
  if (fl_out > 0) {
    digitalWrite(IN7, HIGH);
    digitalWrite(IN8, LOW);
  } else if (fl_out < 0) {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
  }

  fr_error = (fr_w) - fr_rpm;
  fr_cumError += fr_error * deltaT;
  fr_rateError = (fr_error - fr_lastError) / deltaT;
  fr_out = kpfr * fr_error + kifr * fr_cumError + kdfr * fr_rateError;
  
//  Serial.print("fr_w : ");
//  Serial.println(fr_w);
//  Serial.print("fr_error : ");
//  Serial.println(fr_error);
  
  analogWrite(FR_PWM, abs(fr_out));
  fr_lastError = fr_error;

  if (fr_out > 0) {
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
  } else if (fr_out < 0) {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
  }

  bl_error = (bl_w) - bl_rpm;
  bl_cumError += bl_error * deltaT;
  bl_rateError = (bl_error - bl_lastError) / deltaT;
  bl_out = kpbl * bl_error + kibl * bl_cumError + kdbl * bl_rateError;
//  Serial.print("bl_w : ");
//  Serial.println(bl_w);
//  Serial.print("kibl : ");
//  Serial.println(kibl);
//  Serial.print("bl_error : ");
//  Serial.println(bl_error);
//  Serial.print("bl_cumError : ");
//  Serial.println(bl_cumError);
//  Serial.print("bl_out : ");
//  Serial.println(bl_out);
//  Serial.print("bl_rpm : ");
//  Serial.println(bl_rpm);
//  Serial.print("br_w2 : ");
//  Serial.println(br_w);
  
  analogWrite(BL_PWM, abs(bl_out));
  bl_lastError = bl_error;

  if (bl_out > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (bl_out < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  br_error = (br_w) - br_rpm;
  br_cumError += br_error * deltaT;
  br_rateError = (br_error - br_lastError) / deltaT;
  br_out = kpbr * br_error + kibr * br_cumError + kdbr * br_rateError;
//  
//  Serial.print("br_out : ");
//  Serial.println(br_out);
  
  analogWrite(BR_PWM, abs(br_out));
  br_lastError = br_error;

  if (br_out > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (br_out < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}


void front_left_wheel_tick() {
  fl_encCount++;
}

void front_right_wheel_tick() {
  fr_encCount++;
}

void back_left_wheel_tick() {
  bl_encCount++;
}

void back_right_wheel_tick() {
  br_encCount++;
}
