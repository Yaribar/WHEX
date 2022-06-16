// 
// 
// 

#include "H_BRIDGE.h"
#include "PWM_ESP32.h"

#define COAST 0
#define SHORTBRAKE 1

HBRIDGE::HBRIDGE(uint8_t pwm1, uint8_t pwm2, uint8_t channel1, uint8_t channel2, uint8_t bits_resolution)
{
	PWM1.setup(pwm1, channel1, 40000, bits_resolution, true);
	PWM2.setup(pwm2, channel2, 40000, bits_resolution, true);
}

HBRIDGE::~HBRIDGE()
{
}

void HBRIDGE::setSpeed(float speed)
{
	if (speed > 0) {
		PWM1.setDuty(0);
		PWM2.setDuty(abs(speed));
	}
	else if(speed<0){
		PWM1.setDuty(abs(speed));
		PWM2.setDuty(0);
	}
  else{
    PWM1.setDuty(0);
    PWM2.setDuty(0);
  }
}

void HBRIDGE::setStop(bool stop_type)
{
	if (stop_type == SHORTBRAKE) {
		PWM1.setDuty(100);
		PWM2.setDuty(100);
	}
	else {
		PWM1.setDuty(0);
		PWM2.setDuty(0);
	}
}

void HBRIDGE::setFrequency(float frequency)
{
	PWM1.setFrequency(frequency);
	PWM2.setFrequency(frequency);
}
