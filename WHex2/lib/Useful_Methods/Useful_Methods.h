#ifndef _USEFUL_METHODS_h
#define _USEFUL_METHODS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

int8_t sign(int value);
void togglePin(uint8_t pin);
void parseString(String string_in, char* delimiter, float output_array[]);
String buildString(float* data_to_string, uint8_t number_of_elements, char* delimiter);
float getMean(float data[], uint8_t size);
float getVariance(float data[], uint8_t size);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

#endif

