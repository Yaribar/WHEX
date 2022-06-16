#include "filter.h"
#include "Arduino.h"

Filter::Filter()
{
}

Filter::~Filter()
{
}

// void filter::begin(byte _pin)
//{
//    _pin = _pin;
//    
//};
void Filter::setup(float b [], float a [], int m, int n, float xi, float yi)
{
    _b = b;
    _a = a;
    _order[0] = m-1;
    _order[1] = n-1;
    for(int i = 0; i < MAX_ORDER; i++){
        x_aux[i] = xi;
        y_aux[i] = yi;
    }

}

float Filter::filtern(float x)
{
    for (int i = 0; i<= _order[0]; i++){
        x_aux[_order[0]-i] = x_aux[_order[0]-i-1];
    }
    
    for (int i = 0; i<= _order[1]; i++){
       y_aux[_order[1]-i] = y_aux[_order[1]-i-1];
    }

    x_aux[0] = x;
    y_k = 0;

    for (int i=0; i <= _order[0]; i++){
        y_k += _b[i]*x_aux[i];
    }

    for (int i=1; i <= _order[1]; i++){
        y_k -= _a[i]*y_aux[i];
    }

    y_aux[0] = y_k/_a[0];
//    Serial.printf("ORDER A: %d \n", _order[1]);
//    Serial.printf("Y ES: %f,%f,%f,%f,%f \n",y_aux[0], y_aux[1], y_aux[2], y_aux[3], y_aux[4]);
    return y_aux[0];
}