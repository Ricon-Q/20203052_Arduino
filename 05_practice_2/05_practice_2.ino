#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, 1);
}

void loop() {
   delay(1000);
   int toggle = 0;
   for(int i = 0; i < 12; i++){
    toggle = i%2;
    digitalWrite(PIN_LED, toggle);
    delay(100);
   }
   while(1){
    digitalWrite(PIN_LED, toggle);
   }
}
