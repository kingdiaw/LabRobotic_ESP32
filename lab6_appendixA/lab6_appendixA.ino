#include "Arduino.h"
#include "BluetoothSerial.h"
#include "PCF8574.h"  //Library:https://github.com/xreef/PCF8574_library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //By Adafruit Version 2.4.0 Oled 128x32
#include <PixySPI_SS_eps32.h> //https://drive.google.com/file/d/1z82DBqcuNWMVMzOchMi640mwvrnkQEqq/view?usp=sharing 
#include <Ultrasonic.h>       //Ultrasonic by Erick Simões version 3.0.0
#include <Servo.h>       //ServoESP32 by Jaroslav Páral version 1.0.3

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

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

//Setting Parameter for Peripheral
//================================================
//Setting PWM Properties
const int freq = 1000; 
const byte speed1_Channel = 0; 
const byte speed2_Channel = 1;
const byte resolution = 8; 
byte dutyCycle=0;

//Setting OLED Pixels
const byte SCREEN_WIDTH = 128;
const byte SCREEN_HEIGHT = 32;
const byte OLED_RESET = 4;
const byte LINE1 = 0;
const byte LINE2 = 8;
const byte LINE3 = 16;
const byte LINE4 = 24;

//Setting Color Tracking
int signature = 0;
int x = 0;                      //positon x axis
int y = 0;                      //position y axis
unsigned int width = 0;         //object's width
unsigned int height = 0;        //object's height
unsigned int area = 0;
unsigned int newarea = 0;
int Xmin = 130;                  //min x position
int Xmax = 200;                 //max x position
int maxArea = 0;
int minArea = 0;
static int i = 0;

//Mapping Object
//===============================================
// Set i2c address
PCF8574 IC2 (0x20);
BluetoothSerial SerialBT;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
PixySPI_SS pixy;

//================================================
//Global Variable
bool PB1_old = true;
bool PB2_old = true;
bool PB1_new, PB2_new;
bool state;
char line1_buf[32];
char line2_buf[32];
char line3_buf[32];
char line4_buf[32];
unsigned long ledTick;
unsigned long oledTick;
unsigned long previous_time;

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();

  pixy.init();
  
  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
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

  //Set PWM
  ledcSetup(speed1_Channel, freq, resolution);
  ledcSetup(speed2_Channel, freq, resolution);
  ledcAttachPin(ENA, speed1_Channel);
  ledcAttachPin(ENB, speed2_Channel);
  
  oled_print("PRESS PB1 To Continue",0,LINE1);
  oled_print(line2_buf,0,LINE2);
  while(digitalRead(PB1) == HIGH);
  beep();
  oled_clear();
  previous_time = millis(); 
}

void loop()
{
  //Handle Pixy CAM
  //======================================
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[64]; 
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {     
   i++;
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    { 
      signature=pixy.blocks[0].signature;
      x=pixy.blocks[0].x;
      y=pixy.blocks[0].y;
      width = pixy.blocks[0].width;
      height=pixy.blocks[0].height;
      sprintf(line1_buf,"sig:%d x:%d y:%d",signature,x,y);
      sprintf(line2_buf,"w:%d h:%d",width,height);
      
      sprintf(buf, "Detected %d\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();
      }
    }
  } 
  //Handle Blinking LEDs and buzzer
  //======================================
  if(millis()>ledTick){
    ledTick = millis()+500;
    state ^=1;  //toggle it
    IC2.digitalWrite(LED8,state);
    IC2.digitalWrite(LED7,state);
    IC2.digitalWrite(LED6,state);    
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
