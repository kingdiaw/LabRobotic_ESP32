//Mapping IO
const byte PB = 12;
const byte BUZZER = A0;
const byte LED = 13;
const byte pwm1 = 5;
const byte pwm2 = 6;
const byte pwm3 = 9;
const byte m1_dir = A1;
const byte m2_dir = A2;
const byte m3_dir = A3;
byte driver_arr[] = {m1_dir, pwm1, m2_dir, pwm2, m3_dir, pwm3};

//Global Variable
unsigned long ledTick = 0;

//FSM
enum {F1, STOP} State = F1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);
  pinMode(PB, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  for (byte i = 0; i < 6; i++)pinMode(driver_arr[i], OUTPUT);
  Serial.println("Press PB1 to Start!");
  beep();
  while (digitalRead(PB) == HIGH);
}

void loop() {
  // WRITE YOUR CODE HERE
  //========================================
  kiwi(1,255,1,255,1,255); //m1_dir,pwm1,m2_dir,pwm2,m3_dir,pwm3

  //========================================
  if (millis() > ledTick) {
    ledTick = millis() + 200;
    digitalWrite(LED, digitalRead(LED) ^ 1);
  }
}

void beep() {
  digitalWrite(BUZZER, HIGH);
  delay(200);
  digitalWrite(BUZZER, LOW);
  delay(100);
}

void kiwi(byte m1, byte speed1, byte m2, byte speed2, byte m3, byte speed3) {
  if (m1) {
    digitalWrite(m1_dir, HIGH);
  }
  else {
    digitalWrite(m1_dir, LOW);
  }
  analogWrite(pwm1, speed1);
  if (m2) {
    digitalWrite(m2_dir, HIGH);
  }
  else {
    digitalWrite(m2_dir, LOW);
  }
  analogWrite(pwm2, speed2);
  if (m3) {
    digitalWrite(m3_dir, HIGH);
  }
  else {
    digitalWrite(m3_dir, LOW);
  }
  analogWrite(pwm3, speed3);
}
