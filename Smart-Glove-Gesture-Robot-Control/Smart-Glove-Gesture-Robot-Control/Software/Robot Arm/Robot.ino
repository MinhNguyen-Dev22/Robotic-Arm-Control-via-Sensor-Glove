#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50 
#define SERVO_MIN  150 
#define SERVO_MAX  600 

const float MAX_SPEED = 2.0;
const float MAX_ACCEL = 0.12;

float currentAngles[6]     = {60.0, 60.0, 10.0, 90.0, 90.0, 10.0};
float currentVelocities[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float targetAngles[6]      = {60.0, 60.0, 10.0, 90.0, 90.0, 10.0};

unsigned long lastServoUpdate = 0;

typedef struct { 
    float S1; float S2; float S3; float S4; float S5; float S6; 
} RobotCommand;

int angleToPWM(float angle) {
    angle = constrain(angle, 0.0, 180.0);
    return map((int)angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    RobotCommand incoming;
    memcpy(&incoming, incomingData, sizeof(incoming));
    targetAngles[0] = incoming.S1;
    targetAngles[1] = incoming.S2;
    targetAngles[2] = incoming.S3;
    targetAngles[3] = incoming.S4;
    targetAngles[4] = incoming.S5;
    targetAngles[5] = incoming.S6; 
}

void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
    for(int i = 0; i < 6; i++) {
        pwm.setPWM(i, 0, angleToPWM(currentAngles[i]));
    }
    WiFi.mode(WIFI_STA);
    Serial.print("DIA CHI MAC CUA ROBOT LA: ");
    Serial.println(WiFi.macAddress()); 
    if (esp_now_init() != ESP_OK) {
        Serial.println("Loi khoi tao ESP-NOW!");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("Bo dieu toc va gia toc Profiled-Control da san sang!");
}

void loop() {
    if (millis() - lastServoUpdate >= 20) {
        lastServoUpdate = millis();
        for (int i = 0; i < 6; i++) {
            float diff = targetAngles[i] - currentAngles[i];
            float desiredVelocity = diff * 0.15;
            float speedLimit = (i == 5) ? (MAX_SPEED * 1.5) : MAX_SPEED;
            desiredVelocity = constrain(desiredVelocity, -speedLimit, speedLimit);
            float velDiff = desiredVelocity - currentVelocities[i];
            float accelLimit = (i == 5) ? (MAX_ACCEL * 1.5) : MAX_ACCEL;
            velDiff = constrain(velDiff, -accelLimit, accelLimit);
            currentVelocities[i] += velDiff;
            currentAngles[i]     += currentVelocities[i];
            pwm.setPWM(i, 0, angleToPWM(currentAngles[i]));
        }
    }
}