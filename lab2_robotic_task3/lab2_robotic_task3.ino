#include "Arduino.h"
#include "PCF8574.h"  //Library:https://github.com/xreef/PCF8574_library
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //By Adafruit Version 2.4.0 Oled 128x32

//Mapping I/O
//================================================
#define LED8  P7
#define LED7  P6
#define LED6  P5
#define BUZ   P4
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
//Declare FSM
enum {F1,TURN_LEFT,F2,TURN_RIGHT,F3,STOP}State=F1;

//Setting Parameter for Peripheral
//================================================
byte dutyCycle=0;

//Setting OLED Pixels
const byte SCREEN_WIDTH = 128;
const byte SCREEN_HEIGHT = 32;
const byte OLED_RESET = 4;
const byte LINE1 = 0;
const byte LINE2 = 8;
const byte LINE3 = 16;
const byte LINE4 = 24;

//Setting HCSR-4
const int Time_out = 3000;

//Mapping Object
//===============================================
// Set i2c address
PCF8574 IC2 (0x20);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//================================================
//Global Variable
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
unsigned long previous_time;
unsigned long sensorTick;
int distance;

void setup()
{
  Serial.begin(115200);
 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();

  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  // Set IC2 pinMode
  IC2.pinMode(LED8, OUTPUT);
  IC2.pinMode(LED7, OUTPUT);
  IC2.pinMode(LED6, OUTPUT);
  IC2.pinMode(BUZ, OUTPUT); 
  IC2.pinMode(IN1,OUTPUT);
  IC2.pinMode(IN2,OUTPUT);
  IC2.pinMode(IN3,OUTPUT);
  IC2.pinMode(IN4,OUTPUT); 
  IC2.begin();

  IC2.digitalWrite(BUZ, LOW);

  dutyCycle = map(analogRead(VR),0,4096,0,255);
  sprintf(line1_buf,"duty cycle:%d",dutyCycle);
  oled_print(line1_buf,0,LINE1);
  oled_print("PRESS PB1 To Continue",0,LINE2);

  while(digitalRead(PB1) == HIGH);
  beep();
  oled_clear();
  previous_time = millis(); 
}

void loop()
{
  //WRITE YOUR CODE HERE
  //======================================
  if(millis()>sensorTick){
  sensorTick = millis()+50;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO,HIGH,Time_out);
  if ( duration == 0 ) {
	duration = Time_out; }
  distance = duration * 0.034 / 2;  // Calculate the distance in centimeters
  sprintf(line1_buf,"Distance(CM):%d",distance);    
  }
  
  //Task 3-Using FSM
  sprintf(line2_buf,"Current State:%d",State);
  switch(State){
      case F1:
        if(distance >15){
          robot_forward();          
        }
        else{
          robot_stop();
          State = TURN_LEFT;
        }
      break;
      case TURN_LEFT:
          robot_turn_left();
          State = F2;
      break;
      case F2:
         if(distance >15){
          robot_forward();
         }
         else{
          robot_stop();
          State = TURN_RIGHT;
         }
      break;
      case TURN_RIGHT:
          robot_turn_right();
          State = F3;
      break;
      case F3:
          if(distance > 15){
            robot_forward();
          }
          else{
            robot_stop();
            State = STOP;
          }
      break;
      case STOP:
          robot_stop();
      break;  
  }

  //======================================

  //Handle Blinking LEDs and buzzer
  //======================================
  if(millis()>ledTick){
    ledTick = millis()+500;
    led_state ^=1;  //toggle it
    IC2.digitalWrite(LED8,led_state);
    IC2.digitalWrite(LED7,led_state);
    IC2.digitalWrite(LED6,led_state);    
  }
  //=======================================

 //Handle Oled Refresh Display
 //========================================
 if(millis()> oledTick){
  oledTick = millis()+1000;
  oled_clear();
  oled_print(line1_buf,0,LINE1);
  oled_print(line2_buf,0,LINE2);
  oled_print(line3_buf,0,LINE3);
  oled_print(line4_buf,0,LINE4);
 }
 //=======================================
 
}//End loop

//========================================

//User Define Function
//========================================
void oled_print(const char* str, byte col,byte row){
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(col,row);
  display.println(str);
  display.display();
}

void oled_clear(){
  display.clearDisplay();
}

void beep(){
  IC2.digitalWrite(BUZ, HIGH);  
  delay(200);
  IC2.digitalWrite(BUZ, LOW);
}

//Robot movement control function
void robot_forward(){
  
}

void robot_reverse(){
  
}

void robot_stop(){
  
}

void robot_turn_left(){
  
}

void robot_turn_right(){
  
}
