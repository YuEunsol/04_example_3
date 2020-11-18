#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// configurable parameters
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DUTY_MIN 1000 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1600 // servo neutral position (90 degree)
#define _DUTY_MAX 2050 // servo full counterclockwise position (180 degree)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev; // unit: mm
float scale; // used for pulse duration to distance conversion
float dist_ema;
float dist_rail;
float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return val;
  }
Servo myservo;


void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_IR,INPUT);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
//  scale = 0.001 * 0.5 * SND_VEL;
  dist_ema = dist_raw;

// initialize serial port
  Serial.begin(57600);




}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
//  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = ir_distance();
  dist_rail = dist_raw + 70.0;
  
  dist_ema = 0.2*dist_raw + 0.8*dist_ema;

// output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());
  Serial.println(",Max:400");
  Serial.println(dist_rail);



// adjust servo position according to the USS read value



if(dist_rail < 200.0) {
     myservo.writeMicroseconds(_DUTY_MAX);
     analogWrite(PIN_LED, 255);
  }
  else if(200.0 <= dist_rail < 350.0){
     analogWrite(PIN_LED, 0);
     myservo.writeMicroseconds(2050 - (dist_ema-200.0)*7);
     delay(40);
  }

  else if(350.0 <= dist_rail){
    myservo.writeMicroseconds(_DUTY_MIN);
    analogWrite(PIN_LED, 255);
  }
}


//if(dist_rail < 255.0) {
//    myservo.writeMicroseconds(_DUTY_MAX);
//  }
//  else {
//    myservo.writeMicroseconds(_DUTY_MIN);
//  }
