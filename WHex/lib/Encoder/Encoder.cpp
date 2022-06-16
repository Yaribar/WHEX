#include "Encoder.h"

#define SAMPLING_PERIOD 10

Encoder::Encoder()
{

}

Encoder::~Encoder()
{

}

void Encoder::setup(uint8_t channelA,uint8_t channelB){
    _channelA=channelA;
    _channelB=channelB;
    void (*pcb)() = registerCB(this);
    if( pcb ) {
        Serial.print("interrupt");
        pinMode(_channelA, INPUT_PULLUP);
        pinMode(_channelB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(_channelA), pcb, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_channelB), pcb, CHANGE);
    }
    else {
        // No free callback available in the pool.
    }
}

bool Encoder::getDirection(){
    return out;
}

float Encoder::getSpeed(int sampling){

        degrees2 = counter2*360.0/maxSteps;
        _speed =  degrees2/float(sampling);
        counter2=0;
        return _speed;

}

float Encoder::getPosition(){
    degrees = ISRCounter*360.0/maxSteps;
    return degrees;
}

void Encoder::setReference(float reference)
{
  ISRCounter = reference;
}

void Encoder::pollState() {
  // Here is the interrupt handling logic for the class
    old = new_value;
    new_value = digitalRead(_channelA) * 2 + digitalRead(_channelB); // Convert binary input to decimal value
    out = QEM [old * 4 + new_value];
    ISRCounter = ISRCounter + out;
    //if (ISRCounter > maxSteps) ISRCounter=maxSteps;
    //if (ISRCounter < -1*maxSteps) ISRCounter=-1*maxSteps;
    counter2=counter2+out;
/*     if ( counter2 > maxSteps){
        _rev+=1;
        //counter2=0;
    }
    if (counter2 < -1*maxSteps){
        _rev+=1;
        //counter2=0;
    } */

}

callback_t callbacks[] = {
  { p0, NULL },
  { p1, NULL },
  { p2, NULL },
  { p3, NULL },
  { p4, NULL }  // Add more records here...
};

void px( callback_t *p ) { if(p->pobj) p->pobj->pollState(); };
void p0() { px(callbacks + 0); };
void p1() { px(callbacks + 1); };
void p2() { px(callbacks + 2); };
void p3() { px(callbacks + 3); };
void p4() { px(callbacks + 4); }; // Add more functions here...

// Callback alocation function. This is called from within the class object constructor
// to find a free callback function for the interrupt registration.
void (*registerCB(Encoder *pobj))() {
  for( int i=0; sizeof(callbacks)/sizeof(callbacks[0]) ; i++ ) {
    if(!callbacks[i].pobj) {
      callbacks[i].pobj = pobj;
      return callbacks[i].pcb;
    }
  }
  return NULL;
}



