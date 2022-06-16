// H_BRIDGE.h

#ifndef _H_BRIDGE_h
#define _H_BRIDGE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "PWM_ESP32.h"

#define COAST 0
#define SHORTBRAKE 1

class HBRIDGE {
public:
	HBRIDGE(uint8_t pwm1, uint8_t pwm2, uint8_t channel1, uint8_t channel2, uint8_t bits_resolution);
	~HBRIDGE();
	PWM PWM1, PWM2;
	void setSpeed(float speed);
	void setStop(bool stop_type);
	void setFrequency(float frequency);
private:
	float _speed;
	//max_speed;
	bool stop_type = SHORTBRAKE;
};

#endif

