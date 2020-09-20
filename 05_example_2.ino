#define PIN_LED 7

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  while(!Serial){
    ;
  }
  // put your setup code here, to run once:

}

void loop() {
  digitalWrite(PIN_LED, LOW);
  delay(1000);
  for(int i=0;i<5;i++){
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }
  while(1){
    digitalWrite(PIN_LED, HIGH);
    }
  
  // put your main code here, to run repeatedly:

}
