#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <NeoSWSerial.h>
#include <queue.h>

#include "AccelStepper.h"
#include "NeoHWSerial.h"

// DEFINE PIN
#define SPEED_LEFT 5
#define SPEED_RIGHT 6

#define MEGA_SW
#define MEGA
#ifdef MEGA
#ifdef MEGA_SW
#define bluetooth NeoSWSerial(50, 51)
#else 
#define bluetooth NeoSerial1
#endif
#else
#define bluetooth NeoSWSerial(4, 13)
#endif

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
const char CODE_MIN_PWM = 'H';
const char CODE_MAX_PWM = 'I';

// ACCEL DRIVER DEFINE
#define RUN_SPEED 0
#define MAX_SPEED 5000
#define MAX_ACCEL 5000

#define ZERO_VERTI -60000
#define CENTER_VERTI 14000
#define MAX_VERTI 28000

#define ZERO_HORI -25000
#define CENTER_HORI 9750
#define MAX_HORI 19500

AccelStepper angleAtasBawah(AccelStepper::FULL2WIRE, 8, 9);
AccelStepper angleSamping(AccelStepper::FULL2WIRE, 10, 11);
int lastPositionHori = 0;
int lastPositionVerti = 0;

#define RELEASE_PIN 6
#define ISR_PIN_HORI 2
#define ISR_PIN_VERTI 3

// CONST DEFINE
#define MAX_ANALOG 255
#define MAX_PERCENT 100
#define MAX_STEP_APPS 200

int MIN_PWM = 45;
int MAX_PWM = 155;

bool isAnalog = false;
struct dataLog {
    char code;
    int data;
};
#ifdef MEGA
// Pins
const byte Analog_X_pin = A0;  // x-axis readings
const byte Analog_Y_pin = A1;  // y-axis readings

int Analog_X_AVG = 0;  // x-axis value average
int Analog_Y_AVG = 0;  // y-axis value average

#endif
void stopsHori() {
    angleSamping.stop();
    angleSamping.setCurrentPosition(0);
    Serial.println("Current HORI : " + String(angleSamping.currentPosition()));
    detachInterrupt(digitalPinToInterrupt(ISR_PIN_HORI));
}
void stopsVerti() {
    angleAtasBawah.stop();
    angleAtasBawah.setCurrentPosition(0);
    Serial.println("Current VERTI : " + String(angleAtasBawah.currentPosition()));
    detachInterrupt(digitalPinToInterrupt(ISR_PIN_VERTI));
}
void resetPosition() {
    Serial.println("Detect : " + String(digitalRead(ISR_PIN_HORI)));
    if (digitalRead(ISR_PIN_HORI) == 0) {
        angleSamping.setCurrentPosition(0);
        angleSamping.moveTo(CENTER_HORI);
        angleSamping.runToPosition();
    } else {
        attachInterrupt(digitalPinToInterrupt(ISR_PIN_HORI), stopsHori, LOW);
        angleSamping.move(ZERO_HORI);
        while (angleSamping.distanceToGo() < 0) {
            angleSamping.run();
        }
        angleSamping.moveTo(CENTER_HORI);
        angleSamping.runToPosition();
        Serial.println("INITIAL POSITION HORIZONTAL : " + String(angleSamping.currentPosition()));
    }

    Serial.println("Detect VERTI : " + String(digitalRead(ISR_PIN_VERTI)));
    if (digitalRead(ISR_PIN_VERTI) == 0) {
        angleAtasBawah.setCurrentPosition(0);
        angleAtasBawah.moveTo(CENTER_VERTI);
        angleAtasBawah.runToPosition();
    } else {
        attachInterrupt(digitalPinToInterrupt(ISR_PIN_VERTI), stopsVerti, LOW);
        angleAtasBawah.move(ZERO_VERTI);
        while (angleAtasBawah.distanceToGo() < 0) {
            angleAtasBawah.run();
        }
        angleAtasBawah.moveTo(CENTER_VERTI);
        angleAtasBawah.runToPosition();
        Serial.println("INITIAL POSITION VERTICAL : " + String(angleAtasBawah.currentPosition()));
    }
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
            if (command == CODE_SPEED_LEFT || command == CODE_SPEED_RIGHT || command == CODE_ANGLE_VERTICAL || command == CODE_ANGLE_HORIZONTAL || command == CODE_RELEASE || command == CODE_TOGGLE || command == CODE_RESET || command == CODE_MIN_PWM || command == CODE_MAX_PWM) {
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
                    buffer.data = map(buffer.data, 0, 200, MIN_PWM, MAX_PWM);
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
                    buffer.data = map(buffer.data, 0, 200, MIN_PWM, MAX_PWM);
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
                // int run = 0;
                if (buffer.data > MAX_STEP_APPS) {
                    buffer.data = (buffer.data - MAX_STEP_APPS) * -1;
                }
                Serial.println("Buffer Data : " + String(buffer.data));
                int run = map(buffer.data, -200, 200, 0, MAX_HORI);
                angleSamping.moveTo(run);
                angleSamping.runToPosition();
                Serial.println("CODE_ANGLE_HORIZONTAL : " + String(angleSamping.currentPosition()) + " RUN : " + String(run));
                /* code */
                break;
            }

            case CODE_ANGLE_VERTICAL: {
                float runs = 0;
                if (buffer.data > MAX_STEP_APPS) {
                    buffer.data = (buffer.data - MAX_STEP_APPS) * -1;
                }
                Serial.println("Buffer Data : " + String(buffer.data));
                runs = MAX_VERTI - int((70 * (buffer.data + MAX_STEP_APPS)));
                // Serial.println("Buffer Data : " + String(buffer.data));
                // int runs = map(buffer.data, -200, 200, MAX_VERTI, 0);
                // if (runs < 0) {
                //     runs = map(buffer.data, -200, 200, MAX_VERTI, 0);
                // }

                angleAtasBawah.moveTo(runs);
                angleAtasBawah.runToPosition();
                Serial.println("CODE_ANGLE_VERTI : " + String(angleAtasBawah.currentPosition()) + " RUN : " + String(runs));
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
            case CODE_MIN_PWM: {
                MIN_PWM = buffer.data;
                break;
            }
            case CODE_MAX_PWM: {
                MAX_PWM = buffer.data;
                break;
            }
            default:
                break;
            }
        }
    }
}
#ifdef MEGA
void analogTask(void *pvParameters) {
    // Variables
    int Analog_X = 0;  // x-axis value
    int Analog_Y = 0;  // y-axis value
    while (1) {
        // Reading the 3 potentiometers in the joystick: x, y and r.
        Analog_X = analogRead(Analog_X_pin);
        Analog_Y = analogRead(Analog_Y_pin);

        // if the value is 25 "value away" from the average (midpoint), we allow the update of the speed
        // This is a sort of a filter for the inaccuracy of the reading
        if (abs(Analog_X - Analog_X_AVG) > 25) {
            angleSamping.setSpeed(5 * (Analog_X - Analog_X_AVG));
        } else {
            angleSamping.setSpeed(0);
        }
        //----------------------------------------------------------------------------
        if (abs(Analog_Y - Analog_Y_AVG) > 25) {
            angleAtasBawah.setSpeed(5 * (Analog_Y - Analog_Y_AVG));
        } else {
            angleAtasBawah.setSpeed(0);
        }
        angleSamping.runSpeed();  // step the motor (this will step the motor by 1 step at each loop indefinitely)
        angleAtasBawah.runSpeed();
        vTaskDelay(10);
    }
}
void InitialValues() {
    // Set the values to zero before averaging
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
    }
    //----------------------------------------------------------------------------
    Analog_X_AVG = tempX / 50;
    Analog_Y_AVG = tempY / 50;
    //----------------------------------------------------------------------------
    Serial.print("AVG_X: ");
    Serial.println(Analog_X_AVG);
    Serial.print("AVG_Y: ");
    Serial.println(Analog_Y_AVG);
    Serial.println("Calibration finished");
}
#endif
void setup() {
    Serial.begin(9600);
    bluetooth.begin(9600);

#ifndef MEGA
    bluetooth.listen();
#endif
#ifdef MEGA_SW
    bluetooth.listen();
#endif

    pinMode(SPEED_LEFT, OUTPUT);
    pinMode(SPEED_RIGHT, OUTPUT);
    pinMode(ISR_PIN_HORI, INPUT_PULLUP);
    pinMode(ISR_PIN_VERTI, INPUT_PULLUP);
    pinMode(Analog_X_pin, INPUT);
    pinMode(Analog_Y_pin, INPUT);
    angleAtasBawah.setMaxSpeed(MAX_SPEED);
    angleAtasBawah.setAcceleration(MAX_ACCEL * 2);
    angleSamping.setMaxSpeed(MAX_SPEED);
    angleSamping.setAcceleration(MAX_ACCEL);
    resetPosition();
    InitialValues();

    dataQueue = xQueueCreate(10, sizeof(dataLog));
    if (dataQueue == NULL) {
        Serial.println("Queue can not be created");
    }
    xTaskCreate(
        btTask, "btTask",
        256,
        NULL,
        2,
        NULL);

    xTaskCreate(
        actTask, "actTask",
        256,
        NULL,
        2,
        NULL);
#ifdef MEGA
    // xTaskCreate(
    //     analogTask, "analogTask",
    //     128,
    //     NULL,
    //     2,
    //     NULL);
#endif
}

void loop() {
}
