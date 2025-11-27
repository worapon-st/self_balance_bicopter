#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

Servo propL;
Servo propR;

enum DBG_Log { MPU6050, PID_ROLL, PID_PITCH};
enum Axis { ROLL, PITCH, YAW };

const typedef enum {
  ESC1_PIN        = 2,
  ESC2_PIN        = 3,
  SDA_PIN         = 21,
  SCL_PIN         = 22,
} pin;

const typedef enum {
  MPU6050_ADDR    = 0x68,
  MAIN_CONF       = 0x1A,
  GYRO_CONF       = 0x1B,
  ACCEL_CONF      = 0x1C,
  ACCEL_XOUT_H    = 0x3B,
  GYRO_XOUT_H     = 0x43,
  PWR_MGMT_1      = 0x6B,
} reg_addr;

struct GyroInfo {
  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;
  float RateRoll;
  float RatePitch;
  float RateYaw;
  float RateRollCalibrate;
  float RatePitchCalibrate;
  float RateYawCalibrate;
} gyro;

struct AccelInfo {
  int16_t AccXLSB;
  int16_t AccYLSB;
  int16_t AccZLSB;
  float AccX;
  float AccY;
  float AccZ;
} accel;

struct AngleInfo {
  float AngleRoll;
  float AnglePitch;
  float AngleYaw;
} angle;

struct KalmanInfo {
  float AngleRoll;
  float AnglePitch;
  float AngleRollUncertainly;
  float AnglePitchUncertainly;
  float output[];
};
KalmanInfo kalman = {0, 0, 2*2, 2*2, {0, 0}};

float timeCurr, timePrev, elapsedTime;
float desire_angle = 0;
float pwmLeft, pwmRight;
double throttle = 1200;

struct pidInfo {
  float prev_error;
  float error;
  float pid_value;
  float pid_p;
  float pid_i;
  float pid_d;
};
pidInfo roll;
pidInfo pitch;


void mpu6050_conf() {
  // Powering Sensor
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  // Enable DLPF [5]
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MAIN_CONF);
  Wire.write(0x05);
  Wire.endTransmission();

  // Gyroscope Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONF);
  Wire.write(0x08);
  Wire.endTransmission();

  // Accelerometer Config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_CONF);
  Wire.write(0x10);
  Wire.endTransmission();
}

void mpu_gyro() {    
  // Request for Gyro data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(MPU6050_ADDR, 6);
  gyro.GyroX = Wire.read() << 8 | Wire.read();
  gyro.GyroY = Wire.read() << 8 | Wire.read();
  gyro.GyroZ = Wire.read() << 8 | Wire.read();

  // Convert Scale and Turn to "Rotating Rate"
  gyro.RateRoll = (float)gyro.GyroX/65.5;
  gyro.RatePitch = (float)gyro.GyroY/65.5;
  gyro.RateYaw = (float)gyro.GyroZ/65.5;
}

void mpu_accel() {
  // Request for Accel data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(MPU6050_ADDR, 6);
  accel.AccXLSB = Wire.read() << 8 | Wire.read();
  accel.AccYLSB = Wire.read() << 8 | Wire.read();
  accel.AccZLSB = Wire.read() << 8 | Wire.read();

  // Convert Scale
  accel.AccX = (float)accel.AccXLSB/4096 - 0.04; // Add ±number to calibrate
  accel.AccY = (float)accel.AccYLSB/4096 + 0.001;
  accel.AccZ = (float)accel.AccZLSB/4096 + 0.004;

  // Turn to "Angle" (deg)
  float turnDeg = 180/3.14159;
  angle.AngleRoll = atan(accel.AccY/sqrt((accel.AccX*accel.AccX)+(accel.AccZ*accel.AccZ))) * turnDeg;
  angle.AnglePitch = -atan(accel.AccX/sqrt((accel.AccY*accel.AccY)+(accel.AccZ*accel.AccZ))) * turnDeg;
}

void kalman_filter(float state, float uncertainly, float input, float measurement) {
  // (1) Predict current state
  //    : predict value, by using feedback and input.
  //    # feedback and input is same data.
  state = state + (0.004 * input);
  
  // (2) Predict the uncertainly
  //    : find uncertainly of measurement
  float Ts_variance = 0.004*0.004;
  uint8_t gyro_variance = 4*4;
  uncertainly = uncertainly + (Ts_variance * gyro_variance);

  // (3) Compute Kalman Gain
  //    : trust prediction, if measurement is noisy.
  //    : trust measurement, if prediction is uncertain.
  float gain;
  uint8_t accel_variance = 3*3;
  gain = uncertainly * 1/(1 * uncertainly + accel_variance);

  // (4) Update state estimate
  //    : prediction output from actual sensor.
  state = state + gain * (measurement - state);

  // (5) Update uncertainly
  //    : improve correction by decrease uncertainly 
  uncertainly = (1 - gain) * uncertainly;

  kalman.output[0] = state;
  kalman.output[1] = uncertainly;
}

void detail_log(uint8_t dbg_log) {
  switch (dbg_log)
  {
  case MPU6050:
    Serial.printf("\nGyro [R] : %f\t", gyro.RateRoll);
    Serial.printf("[P] : %f\t", gyro.RatePitch);
    Serial.printf("[Y] : %f\t", gyro.RateYaw);
    Serial.printf("Accl [X] : %f\t", accel.AccX);
    Serial.printf("[Y] : %f\t", accel.AccY);
    Serial.printf("[Z] : %f\t", accel.AccZ);
    Serial.printf("Angle [R] : %f\t", kalman.AngleRoll);
    Serial.printf("[P] : %f\t", kalman.AnglePitch);
    break;

  case PID_ROLL:
    Serial.printf("\n[P] : %f\t", roll.pid_p);
    Serial.printf("[I] : %f\t", roll.pid_i);
    Serial.printf("[D] : %f\t", roll.pid_d);
    Serial.printf("[PID] : %f\t", roll.pid_value);
    Serial.printf("[E] : %f\t", roll.error);
    Serial.printf("[PE] : %f\t", roll.prev_error);
    break;

  case PID_PITCH:
    Serial.printf("\n[P] : %f\t", pitch.pid_p);
    Serial.printf("[I] : %f\t", pitch.pid_i);
    Serial.printf("[D] : %f\t", pitch.pid_d);
    Serial.printf("[PID] : %f\t", pitch.pid_value);
    Serial.printf("[E] : %f\t", pitch.error);
    Serial.printf("[PE] : %f\t", pitch.prev_error);
    break;

  default:
    Serial.println("UNDEFINE DEBUGGING LOG");
    break;
  }
}

void pid_control(uint8_t axis, float p, float i, float d) {
  // {
  // 1) Find Current Error
  // 2) Find P
  // 3) Find I
  // 4) Find D
  // 5) Combine PID
  // 6) Adjust Throttle
  // 7) Feedback Error
  // }

  switch (axis)
  {
  case ROLL:
    roll.error = kalman.AngleRoll - desire_angle;

    roll.pid_p = p * roll.error;

    if((roll.error > -3) && (roll.error < 3)) { 
      roll.pid_i = roll.pid_i + (i * roll.error); 
    }

    roll.pid_d = d * ((roll.error - roll.prev_error)/elapsedTime);

    roll.pid_value = roll.pid_p + roll.pid_i + roll.pid_d;

    // Undefine state protection
    // {
    if(roll.pid_value < -1000) { roll.pid_value = -1000; }
    else if(roll.pid_value > 1000) { roll.pid_value = 1000; }
    // }

    pwmLeft = throttle + roll.pid_value;
    pwmRight = throttle - roll.pid_value;

    roll.prev_error = roll.error;

    break;

  case PITCH:
    pitch.error = kalman.AnglePitch - desire_angle;

    pitch.pid_p = p * pitch.error;

    if((pitch.error > -3) && (pitch.error < 3)) { 
      pitch.pid_i = pitch.pid_i + (i * pitch.error); 
    }

    pitch.pid_d = d * ((pitch.error - pitch.prev_error)/elapsedTime);

    pitch.pid_value = pitch.pid_p + pitch.pid_i + pitch.pid_d;

    // Undefine state protection
    // {
    if(pitch.pid_value < -1000) { pitch.pid_value = -1000; }
    else if(pitch.pid_value > 1000) { pitch.pid_value = 1000; }
    // }

    pwmLeft = throttle + pitch.pid_value;
    pwmRight = throttle - pitch.pid_value;

    pitch.prev_error = pitch.error;

    break;

  default:
    break;
  }

  // BLDC PWM Protection
  // {
  pwmLeft = constrain(pwmLeft, 1000, 2000);
  
  pwmRight = constrain(pwmRight, 1000, 2000);

  //Serial.printf("\n%f | %f", pwmLeft, pwmRight);
  // }

  propL.writeMicroseconds(pwmLeft);
  propR.writeMicroseconds(pwmRight);
}

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  mpu6050_conf();

  for(size_t calibrate_n = 0; calibrate_n < 2000; calibrate_n += 1) {
    mpu_gyro();
    gyro.RateRollCalibrate += gyro.RateRoll;
    gyro.RatePitchCalibrate += gyro.RatePitch;
    gyro.RateYawCalibrate += gyro.RateYaw;
  }
  gyro.RateRollCalibrate /= 2000;
  gyro.RatePitchCalibrate /= 2000;
  gyro.RateYawCalibrate /= 2000;

  propL.attach(ESC1_PIN);
  propR.attach(ESC2_PIN);

  timeCurr = millis();

  // ESC Calibrate
  // {
  propL.writeMicroseconds(2000);
  propR.writeMicroseconds(2000);
  delay(5000);
  propL.writeMicroseconds(1000);
  propR.writeMicroseconds(1000);
  // }
}

void loop() {
  timePrev = timeCurr;
  timeCurr = millis();
  elapsedTime = (timeCurr - timePrev)/1000;

  mpu_gyro();
  gyro.RateRoll -= gyro.RateRollCalibrate;
  gyro.RatePitch -= gyro.RatePitchCalibrate;
  gyro.RateYaw -= gyro.RateYawCalibrate;

  mpu_accel();

  kalman_filter(
    kalman.AngleRoll, 
    kalman.AngleRollUncertainly, 
    gyro.RateRoll, 
    angle.AngleRoll
  );
  kalman.AngleRoll = kalman.output[0];
  kalman.AngleRollUncertainly = kalman.output[1];

  kalman_filter(
    kalman.AnglePitch,
    kalman.AnglePitchUncertainly,
    gyro.RatePitch,
    angle.AnglePitch
  );
  kalman.AnglePitch = kalman.output[0];
  kalman.AnglePitchUncertainly = kalman.output[1];

  pid_control(ROLL, 5, 0.001, 0.5);

  detail_log(MPU6050);
}
