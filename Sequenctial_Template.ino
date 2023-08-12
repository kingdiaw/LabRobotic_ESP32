//1)Include Library
#include <Servo.h>

//2) Define Constant Value
const int servo_pin = 9;

//3) Creating Object
Servo myservo;

//4) Creating Sequences
enum {Idle, Task1, Task2} State = Idle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myservo.attach(servo_pin);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch () {
    case Idle:
    //Action
    //Transition
      break;
    case Task1:
    //Action
    //Transition
      break;
    case Task2:
    //Action
    //Transition
      break;
  }
}
