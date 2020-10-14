#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
//LED 점등을 확인하기 위해서 180이 아닌 160으로 설정
#define _DIST_MIN 160 // minimum distance to be measured (unit: mm)
//LED 점등을 확인하기 위해서 360이 아닌 380으로 설정
#define _DIST_MAX 380 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.85
#define _DUTY_MIN 550 // servo full clockwise position (0 degree)
#define _DUTY_MAX 2400 // servo full counterclockwise position (180 degree)
#define _DUTY_NEU (_DUTY_MIN + _DUTY_MAX) / 2 // servo neutral position (90 degree)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  alpha = _DIST_ALPHA;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_ema = alpha*dist_raw + (1-alpha)*dist_ema;

// output the read value to the serial port
  Serial.print("Min:100,Low:180,raw:");
  Serial.print(dist_raw);
  Serial.print(",servo:");
  Serial.print(myservo.read());
  Serial.print(", ema:");
  Serial.print(dist_ema);  
  Serial.println(",High:220,Max:300");

// adjust servo position according to the USS read value

  // add your code here!
  if(dist_raw < 180.0) { 
    myservo.writeMicroseconds(_DUTY_MIN);
    digitalWrite(PIN_LED, 1);
  }
  else if(180.0 <= dist_raw && dist_raw < 360.0){
    myservo.writeMicroseconds(_DUTY_MIN+((dist_raw - 180)*10.27));
    digitalWrite(PIN_LED, 0);
  }
  else {
    myservo.writeMicroseconds(_DUTY_MAX);
    digitalWrite(PIN_LED, 1);
  }
   
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.

  if(reading == 0.0) reading = dist_prev;
  else dist_prev = reading;
  
  return reading;
}
