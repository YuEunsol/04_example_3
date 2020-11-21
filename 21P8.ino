#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// configurable parameters
#define INTERVAL 20 // sampling interval (unit: ms)
#define _DUTY_MIN 1550 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1600 // servo neutral position (90 degree)
#define _DUTY_MAX 2000 // servo full counterclockwise position (180 degree)

#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 450 // maximum distance to be measured (unit: mm)

// global variables
float dist_min, dist_max, dist_raw, dist_rail; // unit: mm
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
  if(millis() < last_sampling_time + INTERVAL) return;  
  dist_raw = ir_distance();
  dist_ema = 0.3*dist_raw + 0.7*dist_ema;
  
  dist_rail = dist_ema - 70.0;


  
// output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(ir_distance());
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());
  Serial.println(",Max:400");
  Serial.println(dist_rail);




  if(0.0 <= dist_rail && dist_rail < 250.0) {
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  else {
    myservo.writeMicroseconds(_DUTY_MIN);
  }


  last_sampling_time += INTERVAL;
}
