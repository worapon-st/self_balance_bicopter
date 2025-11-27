#include <Arduino.h>
#include <Servo.h>

#define ESC_PIN 4
#define MIN_PWM 1000
#define MAX_PWM 2000

Servo motor;

int32_t dataIn = 1000;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("ESC Calibration Program");
    motor.attach(ESC_PIN);
    Serial.println("Send PWM - 2000us");
    motor.writeMicroseconds(MAX_PWM);

    while(!Serial.available());
    Serial.read();

    Serial.println("Send PWM - 1000us");
    motor.writeMicroseconds(MIN_PWM);
    Serial.println("Calibrated !");

    Serial.println("Enter PWM [1000 - 2000] us : ");
}

void loop() {
    if(Serial.available() > 0) {
        dataIn = Serial.parseInt();
        Serial.printf("\nYou enter %d", dataIn);

        if((dataIn >= 1000) && (dataIn <= 2000)) {
            motor.writeMicroseconds(dataIn);
            float speed = (dataIn - 1000) / 10;
            Serial.printf("\nspeed : %.2f %", speed);
        }
        else {
            motor.writeMicroseconds(MIN_PWM);
        }
    }
}