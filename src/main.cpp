#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <NeoSWSerial.h>
#include <queue.h>

#include "AccelStepper.h"

// DEFINE PIN
#define SPEED_LEFT 5
#define SPEED_RIGHT 6

// BT SERIAL DEFINE
NeoSWSerial bluetooth(4, 3);

// QUEUE TASK DEFINE
QueueHandle_t dataQueue;

// CODE MSG DEFINE
// 0-100 -> A100 -> 100%,,,, ?? CODE[NILAI]
#define CODE_SPEED_LEFT 'A'
#define CODE_SPEED_RIGHT 'B'

// 0 - 100 // 100-200// KETAS -> C100
#define CODE_ANGLE_VERTICAL 'C'
#define CODE_ANGLE_HORIZONTAL 'D'
#define CODE_RELEASE 'E'

// ACCEL DRIVER DEFINE
#define RUN_SPEED 0
#define MAX_SPEED 6500
#define MAX_ACCEL 3000
AccelStepper angleHori(AccelStepper::FULL2WIRE, 8, 9);
AccelStepper angleVerti(AccelStepper::FULL2WIRE, 10, 11);
int lastPositionHori = 0;
int lastPositionVerti = 0;

#define RELEASE_PIN 6

struct dataLog {
    char code;
    int data;
};

void btTask(void *pvParameters) {
    dataLog buffer;
    char dataBle[10];
    int idx;
    bool valid = false;
    while (1) {
        if (bluetooth.available() > 0) {
            char command = bluetooth.read();
            if (command == CODE_SPEED_LEFT || command == CODE_SPEED_RIGHT || command == CODE_ANGLE_VERTICAL || command == CODE_ANGLE_HORIZONTAL || command == CODE_RELEASE) {
                buffer.code = command;
                valid = true;
            } else if (command == '\n' && valid) {
                buffer.data = atoi(dataBle);
                xQueueSend(dataQueue, &buffer, portMAX_DELAY);
                bluetooth.flush();
                buffer.code = 'X';
                memset(dataBle, 0, sizeof(dataBle));
                idx = 0;
                valid = false;
                vTaskDelay(100);
            } else if (valid) {
                dataBle[idx] = command;
                idx++;
            }
        }
    }
}
void actTask(void *pvParameters) {
    dataLog buffer;
    int lastValueSpeedLeft = 0;
    int lastvalueSpeedRight = 0;
    while (1) {
        if (xQueueReceive(dataQueue, &buffer, portMAX_DELAY) == pdPASS) {
            switch (buffer.code) {
            case CODE_SPEED_LEFT:
                lastValueSpeedLeft = (buffer.data / 100) * 255;
                if (lastValueSpeedLeft <= buffer.data) {
                    for (int i = lastValueSpeedLeft; i < buffer.data; i++) {
                        analogWrite(SPEED_LEFT, i);
                        delay(5);
                    }
                    lastValueSpeedLeft = buffer.data;
                } else {
                    for (int i = lastValueSpeedLeft; i > buffer.data; i--) {
                        analogWrite(SPEED_LEFT, i);
                        delay(5);
                    }
                    lastValueSpeedLeft = buffer.data;
                }

                Serial.println("SPEPED LEFT : " + String(buffer.data));
                /* code */
                break;
            case CODE_SPEED_RIGHT:
                lastvalueSpeedRight = (buffer.data / 100) * 255;
                if (lastvalueSpeedRight < buffer.data) {
                    for (int i = lastvalueSpeedRight; i < buffer.data; i++) {
                        analogWrite(SPEED_RIGHT, i);
                        vTaskDelay(5);
                    }
                    lastvalueSpeedRight = buffer.data;
                } else {
                    for (int i = lastvalueSpeedRight; i > buffer.data; i--) {
                        analogWrite(SPEED_RIGHT, i);
                        vTaskDelay(5);
                    }
                    lastvalueSpeedRight = buffer.data;
                }

                Serial.println("SPEED RIGHT : " + String(buffer.data));
                /* code */
                break;
            case CODE_ANGLE_HORIZONTAL:
                if (buffer.data <= 100) {
                    buffer.data = (buffer.data / 100) * float(MAX_SPEED);
                } else {
                    buffer.data = (buffer.data - 100) / 100 * float(MAX_SPEED) * -1;
                }
                angleHori.setSpeed(buffer.data);
                Serial.println("CODE_ANGLE_HORIZONTAL : " + String(buffer.data));
                /* code */
                break;
            case CODE_ANGLE_VERTICAL:
                if (buffer.data <= 100) {
                    buffer.data = (buffer.data / 100) * float(MAX_SPEED);
                } else {
                    buffer.data = (buffer.data - 100) / 100 * float(MAX_SPEED) * -1;
                }
                if (buffer.data > MAX_SPEED) {
                    buffer.data = (buffer.data - MAX_SPEED) * -1;
                }
                angleVerti.setSpeed(buffer.data);
                Serial.println("CODE_ANGLE_VERTICAL : " + String(buffer.data));
                /* code */
                break;
            case CODE_RELEASE:
                digitalWrite(RELEASE_PIN, HIGH);
                vTaskDelay(1000);
                digitalWrite(RELEASE_PIN, LOW);
                Serial.println("CODE_RELEASE");
                /* code */
                break;
            default:
                break;
            }
        }
    }
}

void stepTask(void *pvParameters) {
    while (1) {
        angleHori.runSpeed();
        angleVerti.runSpeed();
        vTaskDelay(10);
    }
}

void setup() {
    Serial.begin(9600);
    bluetooth.begin(9600);
    bluetooth.listen();

    pinMode(SPEED_LEFT, OUTPUT);
    pinMode(SPEED_RIGHT, OUTPUT);
    angleHori.setMaxSpeed(MAX_SPEED);
    angleHori.setAcceleration(MAX_ACCEL);
    angleVerti.setMaxSpeed(MAX_SPEED);
    angleVerti.setAcceleration(MAX_ACCEL);
    angleHori.moveTo(100000);
    angleHori.runToPosition();
    angleVerti.moveTo(-100000);
    angleVerti.runToPosition();
    lastPositionHori = angleHori.currentPosition();
    lastPositionVerti = angleVerti.currentPosition();

    dataQueue = xQueueCreate(4, sizeof(dataLog));
    if (dataQueue == NULL) {
        Serial.println("Queue can not be created");
    }
    xTaskCreate(
        btTask, "btTask",
        128,
        NULL,
        2,
        NULL);

    xTaskCreate(
        actTask, "actTask",
        128,
        NULL,
        2,
        NULL);
}

void loop() {
    angleVerti.runSpeed();
    angleHori.runSpeed();
}
