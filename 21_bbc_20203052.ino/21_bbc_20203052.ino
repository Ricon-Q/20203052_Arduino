#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

// configurable parameters
#define _DUTY_MIN 1350 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 1750 // servo full counterclockwise position (180 degree)

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

#define _SERVO_SPEED 500 // servo speed limit (unit: degree/second)
#define INTERVAL 10  // servo update interval

// global variables
unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
int duty_target, duty_curr;
int servo_speed = _SERVO_SPEED;

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _POS_START;
  myservo.writeMicroseconds(duty_curr);
  
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * servo_speed / 180 * INTERVAL / 1000;



// initialize variables for servo update.
  pause_time = 1;
  toggle_interval = (180.0 / servo_speed + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  
  float raw_dist = ir_distance()+30;
  if(millis() < last_sampling_time + INTERVAL) return;

// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(255 > raw_dist) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }

// update servo position
  myservo.writeMicroseconds(duty_curr);

// output the read value to the serial port
  Serial.print("Min:1000,duty_target:");
  Serial.print(duty_target);
  Serial.print(",duty_curr:");
  Serial.print(duty_curr);
  Serial.print(", min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.println(",Max:2000");


  if(raw_dist >= 255){
    duty_target = _DUTY_MIN;
  }
  else {
    duty_target = _DUTY_MAX;
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}
