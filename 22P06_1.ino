#include <Servo.h>
#define PIN_LED 9 
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 100
#define _DIST_MAX 450

// Distance sensor
#define _DIST_ALPHA 0.3

// Servo range
#define _DUTY_MIN 1100
#define _DUTY_NEU 1550
#define _DUTY_MAX 2100




// Servo speed control
#define _SERVO_ANGLE 50.0  
#define _SERVO_SPEED 90.0

// Event periods
#define _INTERVAL_DIST 10 
#define _INTERVAL_SERVO 10
#define _INTERVAL_SERIAL 30 

// PID parameters
#define _KP 0.0 

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_min, dist_max, dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 

bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr; 

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;



void setup() {
// initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO); // attach servo
  pinMode(PIN_LED,OUTPUT); // initialize GPIO pins
  pinMode(PIN_IR,INPUT);


// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  dist_raw = 0.0;
  dist_ema = dist_raw; 

// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180 ) * (_INTERVAL_SERVO / 1000.0); 
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
}
  

void loop() {
/////////////////////
// Event generator //
///////////////////// 

  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
      last_sampling_time_dist += _INTERVAL_DIST;
      event_dist = true;
  }
  
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
      last_sampling_time_servo += _INTERVAL_SERVO;
      event_servo = true;
  }
  
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
      last_sampling_time_serial += _INTERVAL_SERIAL;
      event_serial = true;
  }

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    dist_ema = ir_distance_filtered();

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
    else if(duty_target < _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }
    
    if(275.0 - dist_ema < 0){
       duty_curr = 1550 + (275.0 -dist_ema)*1.45;
    }
    else {
       duty_curr = 1550 - (dist_ema-275.0)*1.34;
    }
  }
  
  if(event_servo) {
    event_servo=false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) {
        duty_curr = duty_target;
      }
     }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) {
        duty_curr = duty_target;
      }
    }
    // update servo position
     myservo.writeMicroseconds(duty_curr);
  }   

  
  if(event_serial) {
    event_serial = false;
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",High:310,Max:2000");
    Serial.println();
  }
}

float ir_distance(void){ // return value unit: mm
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return value;
}

float ir_distance_filtered(void){ // return value unit: mm
  dist_raw = ir_distance() + 40.0;
  if ( dist_raw < 100.0 ) {
    dist_raw = 100.0 ;
  }else if ( 100.0 <= dist_raw && dist_raw <= 430.0) {
    dist_raw = 100.0 + (dist_raw-100.0)*0.76;
  }else {
    dist_raw = 420.0;
  }

  return _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
}
