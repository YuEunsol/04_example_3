#include <Servo.h> 

// Arduino pin assignment
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
#define _DUTY_NEU 1600
#define _DUTY_MAX 2100

// Servo speed control
#define _SERVO_ANGLE 30.0  
#define _SERVO_SPEED 60.0

// Event periods
#define _INTERVAL_DIST 10
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 0.0 

// global variables
float dist_min, dist_max, dist_raw; // unit: mm
unsigned long last_sampling_time; // unit: ms
float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return val;
  }
float dist_ema;
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_IR,INPUT);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  dist_raw = 0.0;
  dist_ema = dist_raw;
  
// initialize serial port
  Serial.begin(57600);
  last_sampling_time = 0;
}


void loop() {
  if(millis() < last_sampling_time + _INTERVAL_DIST) return;
  if ( dist_raw < 100.0 ) {
    dist_raw = 100.0 ;
  }else if ( 100.0 <= dist_raw && dist_raw <= 350.0) {
    dist_raw = 100.0 + (dist_raw-100.0)*1.24;
  }else {
    dist_raw = 410.0;
  }
  dist_raw = ir_distance() + 30.0;
  dist_ema = 0.2*dist_raw + 0.8*dist_ema;

  // output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(ir_distance());
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());
  Serial.println(",Max:450");

  if(0.0 <= dist_ema && dist_ema < 200.0) {
     myservo.writeMicroseconds(_DUTY_MAX);
  }
  else if(200.0 <= dist_ema && dist_ema < 350.0){
     myservo.writeMicroseconds(2100 - (dist_ema-200.0)*6.66);
  }
  else if(350.0 <= dist_ema && dist_ema <= 450.0){
     myservo.writeMicroseconds(_DUTY_MIN);
  }

  last_sampling_time += _INTERVAL_DIST;
}
