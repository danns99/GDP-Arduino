#include <ezButton.h>

const int BUTTON_PIN = 52; // the number of the pushbutton pin
int number_of_states =4;
int status = 1;
int previous_status=1;

const int LED1_pin=22;
const int LED2_pin=24;
const int LED3_pin=26;
const int LED4_pin=28;

ezButton button(BUTTON_PIN);  // create ezButton object that attach to pin 7;

void setup() {
  Serial.begin(9600);         // initialize serial
  button.setDebounceTime(50); // set debounce time to 50 milliseconds

  
pinMode(LED1_pin, OUTPUT);
pinMode(LED2_pin, OUTPUT);
pinMode(LED3_pin, OUTPUT);
pinMode(LED4_pin, OUTPUT);

Serial.println(status);
}

void loop() {
status_check();
  if(status==1){
    status_OFF();
  }
  if(status==2){
    status_1();
  }
  if(status==3){
    status_2();
  }
  if(status==4){
    status_3();
  } 
  delay(50);
}

void status_check(){
    button.loop();
  if(button.isPressed() && status!=1 && status!=number_of_states) {
    previous_status=status;
    status=1;
  }else if(button.isPressed() && status==number_of_states){
    status=1;
    previous_status=1;
  }else if(button.isPressed()){
    status=previous_status+1;
  }
}

void status_OFF(){
    digitalWrite(LED2_pin,LOW);
    digitalWrite(LED3_pin,LOW);
   digitalWrite(LED4_pin,LOW);
    digitalWrite(LED1_pin, HIGH);
    Serial.println("Mode : OFF");
}

void status_1(){
    digitalWrite(LED1_pin, LOW);
    digitalWrite(LED2_pin, HIGH);
    Serial.println("Mode : 1");
}

void status_2(){
    digitalWrite(LED1_pin, LOW);
    digitalWrite(LED3_pin, HIGH);
    Serial.println("Mode : 2");
}

void status_3(){
    digitalWrite(LED1_pin, LOW);
    digitalWrite(LED4_pin, HIGH);
    Serial.println("Mode : 3");
}
