#include <ezButton.h>

const int BUTTON_PIN = 2; // the number of the pushbutton pin
int number_of_states =4;
int status = 1;
int previous_status=1;


ezButton button(BUTTON_PIN);  // create ezButton object that attach to pin 7;

void setup() {
  Serial.begin(9600);         // initialize serial
  button.setDebounceTime(50); // set debounce time to 50 milliseconds

  
pinMode(3, OUTPUT);
pinMode(4, OUTPUT);
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);

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
  delay(5);
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
    digitalWrite(4,LOW);
    digitalWrite(5,LOW);
    digitalWrite(6,LOW);
    digitalWrite(3, HIGH);
    Serial.println("OFF");
}

void status_1(){
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
    Serial.println("1");
}

void status_2(){
    digitalWrite(3, LOW);
    digitalWrite(5, HIGH);
    Serial.println("2");
}

void status_3(){
    digitalWrite(3, LOW);
    digitalWrite(6, HIGH);
    Serial.println("3");
}
