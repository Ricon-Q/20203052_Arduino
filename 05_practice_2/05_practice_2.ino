// LED breathing using analog input (PWM)
#include <math.h>

void setup()
{
  pinMode(9, OUTPUT); // analog output
}

void loop()
{
  float val = (exp(sin(millis()/1000.0*PI)) - 0.36787944)*108.0;
  analogWrite(9, 255 - val);
}
