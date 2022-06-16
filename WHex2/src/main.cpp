#include <Arduino.h>
#include "Encoder.h"
#include "H_BRIDGE.h"
#include "Useful_Methods.h"
#include "PID.h"
#include "PWM_ESP32.h"
#include <PS4Controller.h>

#define AIN1 32 // PWM 1
#define AIN2 25
#define BIN1 26
#define BIN2 27
#define SAMPLING 100
#define SAMPLING_PID 10
#define A1 33 // Entrada ventana 1  encoder, motor 1
#define B1 34 // Entrada ventana 2 encoder,  motor 1
#define A2 14
#define B2 19
#define SERVO1 18
#define SERVO2 5


Encoder EncoderA, EncoderB;
HBRIDGE PuenteA(AIN1, AIN2, 0, 1, 8);
HBRIDGE PuenteB(BIN1, BIN2, 2, 3, 8);
PIDController PIDA;
PIDController PIDB;
PWM Servo;
PWM ServoB;

float reference = 0, referenceB = 0;
float position, positionB;
float speed, speedB;
long prevTime;
long pid_time;
float outPID, outPIDB;
int select, prev_select;
int resolution_sensor = 11;
float time_PID = SAMPLING_PID / 1000;
int freq_Servo = 50;
float servoA = 0, servoB = 0;
float kp = 13.2, ki = 1.0, kd = 9.5;

String input;
float data[5];


void setup()
{
  Serial.begin(115200);
  PuenteA.setSpeed(0);
  PuenteB.setSpeed(0);
  EncoderA.setup(A1, B1);
  EncoderB.setup(A2, B2);
  PIDA.setup(kp, ki, kd, resolution_sensor);
  PIDB.setup(kp, ki, kd, resolution_sensor);
  Servo.setup(SERVO1, 4, freq_Servo, 10, true); // 20ms = 100 % = 1023; 1ms = 0°, 1.5ms = 90°, 2ms =180°
  ServoB.setup(SERVO2, 5, freq_Servo, 10, true);
}

void loop()
{
  if (Serial.available()){
    input = Serial.readStringUntil('\n');
    parseString(input, ",", data);
    select = data[4];
    servoA = data[2];
    servoB = data[3];
    reference = data[0];
    referenceB = data[1];
  }
  // Movimiento Servos
  Servo.setDuty(servoA);
  ServoB.setDuty(servoB);

  // Resetear posición a cero al pasar de control de velocida a control de posición
  if (select != prev_select)
  {
    if (prev_select == 1 && select == 0)
    {
      EncoderA.setReference(0);
      EncoderB.setReference(0);
    }
    prev_select = select;
  }

  // Cálculo del control cada SAMPLING_PID
  if (millis() - pid_time > SAMPLING_PID)
  {
    switch (select)
    {
    case 0: // Posición
      pid_time = millis();
      position = EncoderA.getPosition();
      outPID = PIDA.setControl(position, reference);
      PuenteA.setSpeed(outPID);
      positionB = EncoderB.getPosition();
      outPIDB = PIDB.setControl(positionB, referenceB);
      PuenteB.setSpeed(outPIDB);
      break;

    case 1: // Velocidad
      pid_time = millis();
      outPID = PIDA.setControl(position, reference);
      PuenteA.setSpeed(outPID);
      outPIDB = PIDB.setControl(positionB, referenceB);
      PuenteB.setSpeed(outPIDB);
      break;

    default:
      break;
    }
  }
}
// FIN

/*     if (PS4.Right()) Serial.println("Right Button");
    if (PS4.Down()) Serial.println("Down Button");
    if (PS4.Up()) Serial.println("Up Button");
    if (PS4.Left()) Serial.println("Left Button");

    if (PS4.Square()) Serial.println("Square Button");
    if (PS4.Cross()) Serial.println("Cross Button");
    if (PS4.Circle()) Serial.println("Circle Button");
    if (PS4.Triangle()) Serial.println("Triangle Button"); */