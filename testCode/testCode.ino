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
#define SERVO 15    //J4
#define SERVO2  33  //J5

//================================================

//Interrupt Mapping
//================================================
#define ESP32_INTERRUPTED_PIN 26

//Function ISR
void readSensorArray();
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

//Setting ADC
const int ADC_Max = 4096;

//Setting RC Servo Motor
const byte minDegree=0;
const byte maxDegree=180;


//Mapping Object
//===============================================
// Set i2c address
PCF8574 IC1 (0x21,ESP32_INTERRUPTED_PIN,readSensorArray);
PCF8574 IC2 (0x20);
BluetoothSerial SerialBT;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
PixySPI_SS pixy;
Ultrasonic ultrasonic(TRIG,ECHO);
Servo myservo;  // create servo object to control a servo

//================================================
//Global Variable
bool sensorDetected = false;
bool PB1_old = true;
bool PB2_old = true;
bool PB1_new, PB2_new;
bool state;
char line1_buf[32];
char line2_buf[32];
char line3_buf[32];
char line4_buf[32];
unsigned char command, command_old;
unsigned long ledTick;
unsigned long adcTick;
unsigned long hcsr04Tick;
unsigned long oledTick;
unsigned long servoTick;

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();

  //Initialize Pixy MCUcam5
  pixy.init();

  //Initialize Ultrasonic HC-SR04 module
  ultrasonic.setTimeout(40000UL);

  //Initialize Servo Library
  myservo.attach(SERVO);
  
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
  IC2.pinMode(IN1,OUTPUT);
  IC2.pinMode(IN2,OUTPUT);
  IC2.pinMode(IN3,OUTPUT);
  IC2.pinMode(IN4,OUTPUT); 
  IC1.begin();
  IC2.begin();

  IC2.digitalWrite(BUZ, LOW);

  //Set PWM
  ledcSetup(speed1_Channel, freq, resolution);
  ledcSetup(speed2_Channel, freq, resolution);
  ledcAttachPin(ENA, speed1_Channel);
  ledcAttachPin(ENB, speed2_Channel);
  
}

void loop()
{
  //Handle Pixy CAM
  //======================================
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
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
      sprintf(line3_buf,"sig:%d",pixy.blocks[i].signature);
      sprintf(line4_buf,"x:%d y:%d",pixy.blocks[i].x,pixy.blocks[i].y);      
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();
      }
    }
  } 

  //Handle Servo
  //=============================================
  if(millis() > servoTick){
    uint16_t Dout;
    servoTick = millis()+200;
    Dout = analogRead(VR);
    Dout = map(Dout, 0, ADC_Max, minDegree, maxDegree);
    myservo.write(Dout); 
  }

  //=============================================

  //Handle HCSR04 Ultrasonic Sensor
  //=============================================
  if(millis()>hcsr04Tick){
    char buf[32];
    uint8_t jarak;
    hcsr04Tick = millis()+1000;
    jarak = ultrasonic.read();
    Serial.print("Distance in CM:");
    Serial.println(jarak);
    sprintf(line2_buf,"DISTANCE:%dCM",jarak);    
  }
  //=============================================

  
  //Handle Serial-Bluetooh Comm
  //======================================
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    command = SerialBT.read();
    Serial.write(command);   
  }  
  //======================================

  //Handle command received
  //======================================
  if(command_old != command){
    if(command == 'f'){
      SerialBT.println("M1 LEFT FORWARD");
      IC2.digitalWrite(IN1, HIGH);
      IC2.digitalWrite(IN2, LOW);
      SerialBT.println("M2 RIGHT FORWARD");
      IC2.digitalWrite(IN3,HIGH);
      IC2.digitalWrite(IN4,LOW);
      dutyCycle = 255;
      ledcWrite(speed1_Channel, dutyCycle);
      ledcWrite(speed2_Channel, dutyCycle);      
    }
    else if(command == 'b'){
      SerialBT.println("M1 LEFT BACKWARD");
      IC2.digitalWrite(IN1, LOW);
      IC2.digitalWrite(IN2, HIGH);
      SerialBT.println("M2 RIGHT BACKWARD");
      IC2.digitalWrite(IN3,LOW);
      IC2.digitalWrite(IN4,HIGH);
      dutyCycle = 255;
      ledcWrite(speed1_Channel, dutyCycle);
      ledcWrite(speed2_Channel, dutyCycle);      
    }
    else{
      SerialBT.println("M1 LEFT STOP");
      IC2.digitalWrite(IN1, LOW);
      IC2.digitalWrite(IN2, LOW);
      SerialBT.println("M2 RIGHT STOP");
      IC2.digitalWrite(IN3,LOW);
      IC2.digitalWrite(IN4,LOW);
      dutyCycle = 0;
      ledcWrite(speed1_Channel, dutyCycle);
      ledcWrite(speed2_Channel, dutyCycle);      
    }
    command_old = command;
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

  //Handle Read PB state
  //=======================================
 PB1_new = digitalRead(PB1);
 PB2_new = digitalRead(PB2);
 if(PB1_old != PB1_new){
  PB1_old = PB1_new;
    if(PB1_new == LOW){
      Serial.println("PB1 Pressed!");
    }
 }
 if(PB2_old != PB2_new){
  PB2_old = PB2_new;
    if(PB2_new == LOW){
      Serial.println("PB2 Pressed!");
    }
 }
 //==========================================

 //Handle ADC
 //==========================================
 if(millis()> adcTick){
  char strbuf[32];
  adcTick = millis()+500;
  int adcRes = analogRead(VR);
  sprintf(line1_buf,"ADC:%d",adcRes);
 }

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

 //Handle IC1 Pin Change (Interrupt)
 //========================================
 if(sensorDetected){
//   Serial.print("Sensor Array:");
//   Serial.print(IC1.digitalRead(S1));
//   Serial.print(IC1.digitalRead(S2));
//   Serial.print(IC1.digitalRead(S3));
//   Serial.print(IC1.digitalRead(S4));
//   Serial.println(IC1.digitalRead(S5));
   sensorDetected = false;
   }
 //=======================================
 
}

//Interrupt Service Routine
//========================================
void readSensorArray(){
  sensorDetected = true;
}
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
