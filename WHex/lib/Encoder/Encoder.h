#ifndef _Encoder_h
#define _Encoder_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
// This code was improve thanks to the forum https://github.com/arduino/ArduinoCore-avr/issues/85
// and https://github.com/ghrasko

#define PPR 48
#define REDUCTION 47

class Encoder
{
public:
  Encoder();
  ~Encoder();
  void setup(uint8_t channelA, uint8_t channelB);
  bool getDirection();
  float getSpeed(int sampling);
  float getPosition();
  void setReference(float reference);
  void pollState(); // Class callback to be finally triggered by the interrupt

private:
  uint8_t _channelA;
  uint8_t _channelB;
  char _resolution;
  const uint16_t maxSteps = PPR * REDUCTION;
  float _speed;
  bool _dir;
  long ISRCounter;
  long counter2;
  int8_t QEM[16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0};
  uint8_t old = 0, new_value = 0;
  int8_t out;
  unsigned long _prevTime = 0;
  float _rev;
  float degrees;
  float degrees2;
};

typedef struct callback
{                // callback table record structure
  void (*pcb)(); // pointer to pool callback routine
  Encoder *pobj; // place for object instance "this" pointer
} callback_t;
void px(callback_t *p);              // error catching wrapper for callbacks
void p0(), p1(), p2(), p3(), p4();   // callback routine pool
void (*registerCB(Encoder *pobj))(); // callback routine allocation routine

#endif
