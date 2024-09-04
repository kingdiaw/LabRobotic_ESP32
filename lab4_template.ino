#include "Arduino.h"
#include "PCF8574.h"  //Library:https://github.com/xreef/PCF8574_library
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //By Adafruit Version 2.4.0 Oled 128x32
//#include <Ultrasonic.h>       //By Erick Simoes Version 3.0.0
//#include <PID_v1.h>           //By Brett Beauregard
#include "BluetoothSerial.h"

//Mapping I/O
//================================================
#define LED8  P7
#define LED7  P6
#define LED6  P5
#define BUZ   P4
#define S1    P0
#define S2    P1
#define S3    P2
#define S4    P3
#define S5    P4
#define PB1   39
#define PB2   34
#define ENA   25
#define ENB   14
#define IN1   P0
#define IN2   P1
#define IN3   P2
#define IN4   P3
#define VR    32
#define TRIG  12
#define ECHO  35
//================================================

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Declare FSM
//===============================================
enum {F1, STOP} State = F1;

//PID Parameter
//===============================================
//Define Variables we'll be connecting to
//double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Setting Parameter for Peripheral
//================================================
//Setting PWM Properties
byte dutyCycle = 0;

//Setting OLED Pixels
const byte SCREEN_WIDTH = 128;
const byte SCREEN_HEIGHT = 32;
const byte OLED_RESET = 4;
const byte LINE1 = 0;
const byte LINE2 = 8;
const byte LINE3 = 16;
const byte LINE4 = 24;

//Setting Motor
const byte base_speed_left=100;
const byte base_speed_right = 115;

//Mapping Object
//===============================================
// Set i2c address
PCF8574 IC1 (0x21);
PCF8574 IC2 (0x20);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Ultrasonic ultrasonic (TRIG, ECHO);
BluetoothSerial SerialBT;

//================================================
//Global Variable
bool sensorDetected = false;
bool PB1_old = true;
bool PB2_old = true;
bool PB1_new, PB2_new;
bool led_state;
char line1_buf[32];
char line2_buf[32];
char line3_buf[32];
char line4_buf[32];
unsigned long ledTick;
unsigned long oledTick;
unsigned long senTick;
unsigned long previous_time;
byte s[5];
byte sensorArray, sensorArrayOld;
int bias;
unsigned char command, command_old;

void setup()
{
  Serial.begin(115200);

  SerialBT.begin("CustomYourBT-name"); //Bluetooth device name

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();

  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //Set IC1 pinMode
  IC1.pinMode(S1, INPUT_PULLUP);
  IC1.pinMode(S2, INPUT_PULLUP);
  IC1.pinMode(S3, INPUT_PULLUP);
  IC1.pinMode(S4, INPUT_PULLUP);
  IC1.pinMode(S5, INPUT_PULLUP);
  // Set IC2 pinMode
  IC2.pinMode(LED8, OUTPUT);
  IC2.pinMode(LED7, OUTPUT);
  IC2.pinMode(LED6, OUTPUT);
  IC2.pinMode(BUZ, OUTPUT);
  IC2.pinMode(IN1, OUTPUT);
  IC2.pinMode(IN2, OUTPUT);
  IC2.pinMode(IN3, OUTPUT);
  IC2.pinMode(IN4, OUTPUT);

  IC1.begin();
  IC2.begin();

  IC2.digitalWrite(BUZ, LOW);

  dutyCycle = map(analogRead(VR), 0, 4096, 0, 255);
  sprintf(line1_buf, "duty cycle:%d", dutyCycle);
  oled_print(line1_buf, 0, LINE1);
  oled_print("PRESS PB1 To Continue", 0, LINE2);

  while (digitalRead(PB1) == HIGH);
  beep();
  oled_clear();
  previous_time = millis();
}

void loop()
{
  //WRITE YOUR CODE HERE
  //======================================
  //Handle Serial-Bluetooh Comm
  //======================================
  if (SerialBT.available()) {
    command = SerialBT.read();
    sprintf(line4_buf,"%c",command);   
  } 
  //Handle command received
  //======================================
  if(command_old != command){
    if(command == 'a'){
    //Perform Task a here
    
    }
    else if(command == 'b'){
    //Perform Task b here 
    
    }
    command_old = command;
  }  
  //======================================

  //Handle Blinking LEDs and buzzer
  //======================================
  if (millis() > ledTick) {
    ledTick = millis() + 500;
    led_state ^= 1; //toggle it
    IC2.digitalWrite(LED8, led_state);
    IC2.digitalWrite(LED7, led_state);
    IC2.digitalWrite(LED6, led_state);
  }
  //=======================================

  //Handle Oled Refresh Display
  //========================================
  if (millis() > oledTick) {
    oledTick = millis() + 1000;
    oled_clear();
    oled_print(line1_buf, 0, LINE1);
    oled_print(line2_buf, 0, LINE2);
    oled_print(line3_buf, 0, LINE3);
    oled_print(line4_buf, 0, LINE4);
  }
  //=======================================

}//End loop

//========================================

//User Define Function
//========================================
void oled_print(const char* str, byte col, byte row) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(col, row);
  display.println(str);
  display.display();
}

void oled_clear() {
  display.clearDisplay();
}

void beep() {
  IC2.digitalWrite(BUZ, HIGH);
  delay(200);
  IC2.digitalWrite(BUZ, LOW);
}

//Robot movement control function
void robot_forward() {
  robot(1,1,base_speed_left,base_speed_right);
}

void robot_reverse() {

}

void robot_stop() {
  robot(1,1,0,0);
}

byte robot_turn_left() {
  if(!IC1.digitalRead(S1)){ //wait until 0xxxx
    robot_stop();
    return 0; 
  }
  robot (0,1,dutyCycle,dutyCycle);
  return 1;
}

byte robot_turn_right() {  
  if(!IC1.digitalRead(S5)){ //wait until xxxx0
    robot_stop();
    return 0; 
  }
  robot (1,0,dutyCycle,dutyCycle);
  return 1;    
}


void robot(unsigned char mLeft, unsigned char mRight, int sLeft, int sRight) {
  if (mLeft > 0) {
    IC2.digitalWrite(IN1, LOW);
    IC2.digitalWrite(IN2, HIGH);
  }
  else {
    IC2.digitalWrite(IN1, HIGH);
    IC2.digitalWrite(IN2, LOW);
  }
  if (mRight > 0) {
    IC2.digitalWrite(IN3, LOW);
    IC2.digitalWrite(IN4, HIGH);
  }
  else {
    IC2.digitalWrite(IN3, HIGH);
    IC2.digitalWrite(IN4, LOW);
  }
  analogWrite(ENA, sLeft);
  analogWrite(ENB, sRight);
}
