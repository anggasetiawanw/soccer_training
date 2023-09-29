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
const char CODE_ANGLE_HORIZONTAL = 'D';
const char CODE_ANGLE_VERTICAL = 'C';
const char CODE_RELEASE = 'E';
const char CODE_TOGGLE = 'F';
const char CODE_RESET = 'G';

// ACCEL DRIVER DEFINE
#define RUN_SPEED 0
#define MAX_SPEED 5000
#define MAX_ACCEL 5000
#define ZERO_VERTI -40000
#define CENTER_VERTI 23000

#define ZERO_HORI -18200
#define CENTER_HORI 9100

AccelStepper angleAtasBawah(AccelStepper::FULL2WIRE, 8, 9);
AccelStepper angleSamping(AccelStepper::FULL2WIRE, 10, 11);
int lastPositionHori = 0;
int lastPositionVerti = 0;

#define RELEASE_PIN 6

// CONST DEFINE
#define MAX_ANALOG 255
#define MAX_PERCENT 100
#define MAX_STEP_APPS 200
#define STEP_HORI 10
#define STEP_VERTI 75

bool isAnalog = false;
// Pins
const byte Analog_X_pin = A0;  // x-axis readings
const byte Analog_Y_pin = A1;  // y-axis readings
const byte Analog_R_pin = A2;  // r-axis readings

// Variables

int Analog_R_Value = 0;  // this is used for the PWM value

struct dataLog {
    char code;
    int data;
};
void resetPosition() {
    angleAtasBawah.moveTo(ZERO_VERTI);
    angleAtasBawah.runToPosition();
    angleAtasBawah.setCurrentPosition(0);
    angleAtasBawah.moveTo(CENTER_VERTI);
    angleAtasBawah.runToPosition();
    angleSamping.moveTo(ZERO_HORI);
    angleSamping.runToPosition();
    angleSamping.setCurrentPosition(0);
    angleSamping.moveTo(CENTER_HORI);
    angleSamping.runToPosition();
    angleSamping.setCurrentPosition(0);
}

void btTask(void *pvParameters) {
    dataLog buffer;
    char dataBle[10];
    int idx;
    bool valid = false;
    while (1) {
        if (bluetooth.available() > 0) {
            char command = bluetooth.read();
            Serial.println("BT READ : " + String(command));
            if (command == CODE_SPEED_LEFT || command == CODE_SPEED_RIGHT || command == CODE_ANGLE_VERTICAL || command == CODE_ANGLE_HORIZONTAL || command == CODE_RELEASE || command == CODE_TOGGLE || command == CODE_RESET) {
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
            case CODE_SPEED_LEFT: {
                if (buffer.data != 0)
                    buffer.data = map(buffer.data, 0, 200, 45, 155);
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
            }
            case CODE_SPEED_RIGHT: {
                if (buffer.data != 0)
                    buffer.data = map(buffer.data, 0, 200, 45, 155);
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
            }

            case CODE_ANGLE_HORIZONTAL: {
                int run = 0;
                if (buffer.data > MAX_STEP_APPS) {
                    buffer.data = (buffer.data - MAX_STEP_APPS) * -1;
                }
                if (lastPositionHori != 0) {
                    run = (lastPositionHori - buffer.data);
                } else {
                    run = buffer.data;
                }
                if (buffer.data < lastPositionHori) {

                    if (run > 0) {
                        run *= -1;
                    }
                } else {
                    if (run < 0) {
                        run *= -1;
                    }
                }
                lastPositionHori = buffer.data;
                angleSamping.runToNewPosition(run * STEP_VERTI);
                Serial.println("CODE_ANGLE_HORIZONTAL : " + String(lastPositionHori) + " RUN : " + String(run));
                /* code */
                break;
            }

            case CODE_ANGLE_VERTICAL: {
                int runs = 0;
                if (buffer.data > MAX_STEP_APPS) {
                    buffer.data = (buffer.data - MAX_STEP_APPS) * -1;
                }
                if (lastPositionVerti != 0) {
                    runs = (lastPositionVerti - buffer.data);
                } else {
                    runs = buffer.data;
                }
                if (buffer.data < lastPositionVerti) {
                    if (runs > 0) {
                        runs *= -1;
                    }
                } else {
                    if (runs < 0) {
                        runs *= -1;
                    }
                }
                lastPositionVerti = buffer.data;
                angleAtasBawah.runToNewPosition(runs * STEP_HORI);
                Serial.println("CODE_ANGLE_VERTICAL : " + String(lastPositionVerti) + " runs : " + String(runs));
                break;
            }

            case CODE_RELEASE: {
                digitalWrite(RELEASE_PIN, HIGH);
                vTaskDelay(1000);
                digitalWrite(RELEASE_PIN, LOW);
                Serial.println("CODE_RELEASE");
                /* code */
            }

            break;
            case CODE_RESET: {
                resetPosition();
                Serial.println("RESET");
                break;
            }
            case CODE_TOGGLE: {
                if (buffer.data == 1) {
                    isAnalog = false;
                    Serial.println("ANALOG FALSE");
                }
                if (buffer.data == 2) {
                    isAnalog = true;
                    Serial.println("ANALOG TRUE");
                }
                break;
            }
            default:
                break;
            }
        }
    }
}

void analogTask(void *pvParameter) {
    int Analog_X = 0;  // x-axis value
    int Analog_Y = 0;  // y-axis value
    int Analog_R = 0;  // r-axis value

    int Analog_X_AVG = 0;  // x-axis value average
    int Analog_Y_AVG = 0;  // y-axis value average
    int Analog_R_AVG = 0;  // r-axis value average
    float tempX = 0;
    float tempY = 0;
    float tempR = 0;
    //----------------------------------------------------------------------------
    // read the analog 50x, then calculate an average.
    // they will be the reference values
    for (int i = 0; i < 50; i++) {
        tempX += analogRead(Analog_X_pin);
        delay(10);  // allowing a little time between two readings
        tempY += analogRead(Analog_Y_pin);
        delay(10);
        tempR += analogRead(Analog_R_pin);
        delay(10);
    }
    //----------------------------------------------------------------------------
    Analog_X_AVG = tempX / 50;
    Analog_Y_AVG = tempY / 50;
    Analog_R_AVG = tempR / 50;
    //-------------------------------
    Serial.println("Calibration finished");
    for (;;) {
        if (isAnalog) {
            Analog_X = analogRead(Analog_X_pin);
            Analog_Y = analogRead(Analog_Y_pin);
            Analog_R = analogRead(Analog_R_pin);

            // if the value is 25 "value away" from the average (midpoint), we allow the update of the speed
            // This is a sort of a filter for the inaccuracy of the reading
            if (abs(Analog_X - Analog_X_AVG) > 25) {
                angleAtasBawah.setSpeed(10 * (Analog_X - Analog_X_AVG));
            } else {
                angleAtasBawah.setSpeed(0);
                Serial.println("0");
            }
            //----------------------------------------------------------------------------
            if (abs(Analog_Y - Analog_Y_AVG) > 25) {
                angleSamping.setSpeed(10 * (Analog_Y - Analog_Y_AVG));
            } else {
                angleSamping.setSpeed(0);
            }
            angleSamping.runSpeed();  // step the motor (this will step the motor by 1 step at each loop indefinitely)
            angleAtasBawah.runSpeed();
        }
        vTaskDelay(50);
    }
}

void setup() {
    Serial.begin(9600);
    bluetooth.begin(9600);
    bluetooth.listen();
    pinMode(Analog_X_pin, INPUT);
    pinMode(Analog_Y_pin, INPUT);
    pinMode(Analog_R_pin, INPUT);
    // pinMode(LED_pin, OUTPUT);
    pinMode(SPEED_LEFT, OUTPUT);
    pinMode(SPEED_RIGHT, OUTPUT);
    angleAtasBawah.setMaxSpeed(MAX_SPEED);
    angleAtasBawah.setAcceleration(MAX_ACCEL * 2);
    angleSamping.setMaxSpeed(MAX_SPEED);
    angleSamping.setAcceleration(MAX_ACCEL);
    resetPosition();

    dataQueue = xQueueCreate(10, sizeof(dataLog));
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
        32,
        NULL,
        2,
        NULL);

    // xTaskCreate(
    //     analogTask, "analogTask",
    //     128,
    //     NULL,
    //     1,
    //     NULL);
}

void loop() {
    // if (isAnalog) {
    //     // ReadAnalog();
    //     angleSamping.runSpeed();  // step the motor (this will step the motor by 1 step at each loop indefinitely)
    //     angleAtasBawah.runSpeed();
    // }
    // vTaskDelay(10);
}
