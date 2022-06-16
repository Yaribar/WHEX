#ifndef _PID_H
#define _PID_H
#include <filter.h>

#define OBENEN_GRENZEN 50   // Limite de error
#define UNTEREN_GRENZEN -50 // limite de error
#define EPSILON 0.005F
#define FANFANG 10000.0F // Indicador de primer ciclo

class PIDController
{

public:
    Filter Filtro;
    PIDController();
    ~PIDController();
    float b[2] = {0.1367, 0.1367};
    float a[2] = {1, -0.7265};
    float _fehler; // Error between reference and actual output
    float _kp;
    float _ki;
    float _kd;
    int ausloesung;
    void setup(
        float kp,
        float ki,
        float kd,
        int sensorausloesung = 0,
        float referenz = 0,  // Desired output value (aka reference)
        float fehler = 0.0F // Error
        );

    float setControl(
        float eingabe, // Input
        float referenz // Desired output value (aka reference)
    );

    float avoidWindUp(
        float wert // Value
    );

    bool TorF(
        float A,
        float B,
        float epsilon = 0.005F);

    void setPIDGains(
        float kp,
        float ki,
        float kd);

    float getPIDGains();

private:
    float _letzte_zeit = 0.0F;        // Last time
    float _letzte_ausgang = 10000.0F; // Last output
    float _letzte_eingabe = 10000.0F; // Last input
    float _letzter_fehler = 0.0F;
    float _i_fehler = 0.0F;
    float _d_fehler = 0.0F;
    float _ausgang = 0.0F;
    float _anteilen[3]; // PID gains array
    float _referenz;
    float _p;
    float _I;
    float _D;
    float _d_eingabe;
    float _abtastzeit;
};
#endif