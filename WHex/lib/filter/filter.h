#ifndef _FILTER_H
#define _FILTER_H

// Definición de variables
#define MAX_ORDER 4
// Clase: filter

class Filter
{
public:
    // byte _pin;
    Filter();
    ~Filter();
    // void begin(byte _pin);
    void setup(float b [], float a [], int m, int n, float xi = 0, float yi = 0);
    float filtern(float x);


private:
    float *_b;
    float *_a;
    float x_aux[MAX_ORDER];
    float y_aux[MAX_ORDER];
    unsigned char _order[2];
    float y_k;
};
#endif