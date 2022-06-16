#include "PID.h"
#include <cmath>
#include <Arduino.h>

PIDController::PIDController()
{
}

PIDController::~PIDController()
{
}

void PIDController::setup(float kp, float ki, float kd, int sensorausloesung, float referenz, float fehler)
{
    _referenz = referenz;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _fehler = fehler;
    Filtro.setup(b, a, 1, 1);
    ausloesung = pow(2,sensorausloesung)-1;
    
}

float PIDController::avoidWindUp(float wert)
{
    if (wert > OBENEN_GRENZEN)
    {
        return OBENEN_GRENZEN;
    }
    else if (wert < UNTEREN_GRENZEN)
    {
        return UNTEREN_GRENZEN;
    }
    return wert;
}

bool PIDController::TorF(float A, float B, float epsilon)
{
    return (fabs(A - B) < epsilon);
}

void PIDController::setPIDGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

float PIDController::getPIDGains()
{
    return _kp, _ki, _kd;
}

float PIDController::setControl(float eingabe, float referenz)
{
    // First loop
    if (TorF(_letzte_eingabe, FANFANG))
    {
        _letzte_eingabe = eingabe;
    }

    // Error computation
    _fehler = referenz - eingabe;
    _d_eingabe = eingabe - _letzte_eingabe;

    // Proportional
    _p = _kp * _fehler;

    // Integral
    _i_fehler += _fehler;
    _I = _ki * _i_fehler;
    _I = avoidWindUp(_I);

    // Derivative
    _d_fehler = (_fehler - _letzter_fehler)/0.01;
    _d_fehler = Filtro.filtern(_d_fehler);
    _D = _kd * _d_fehler;
    // PID together
    _ausgang = _p + _I + _D;

    // Required data for next loop
    _letzter_fehler = _fehler;
    _letzte_ausgang = _ausgang;

    // Serial.printf("error:%0.3f, salida: %0.3f, ult. error: %0.3f %0.3f, %0.3f, %0.3f \n", _fehler, _ausgang, _letzter_fehler,_p, _I, _D);

/*     if (-2.0 < _fehler && _fehler < 2.0)
    {
        _ausgang = 0;
    } */
        _ausgang = _ausgang*100/ausloesung;

    return _ausgang;
}
