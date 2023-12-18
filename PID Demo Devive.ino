#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <LiquidCrystal_I2C.h>
#include  <Wire.h>
LiquidCrystal_I2C lcd(0x27,  16, 2);


#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE
#define PWM 10
#define IN2 8
#define IN1 7
volatile bool isAutomaticMode = true;
const int buttonPin = 4;           // Button pin to switch between manual and automatic modes
const int potPinKp = A0;           // Potentiometer pin for Kp
const int potPinKi = A1;           // Potentiometer pin for Ki
const int potPinKd = A2;           // Potentiometer pin for Kd
volatile int posi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;
const int automaticLedPin = 5;    
const int manualLedPin = 6;  
// Pre-defined PID parameters
double autoKp = 1.0;
double autoKi = 0.1;
double autoKd = 0.01;

float lcdUpdateT;
bool flag=0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(buttonPin, INPUT);  //  INPUT for pull-up resistor
  digitalWrite(buttonPin, HIGH); // turn on pullup resistors
  pinMode(automaticLedPin, OUTPUT);
  pinMode(manualLedPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  lcd.init();
  lcd.backlight();
  lcd.print("PID Demo Device");
  delay(1000);
  lcd.clear();
  Serial.println("target pos");
}
const int potPinTarget = A3;       // Potentiometer pin for target position
float PotCalb = 1.19; //this is used to overcome the issue of the pot never get the angle 360 on real life
// PID constants
float autokp = 1;
float autokd = 0.025;
float autoki = 0.0;
float kp = autokp;
float ki = autoki;
float kd = autokd;
void loop() {

  isAutomaticMode = !digitalRead(buttonPin);
  if(!isAutomaticMode){
    kp = PotCalb * map(analogRead(potPinKp), 0, 1023, 0, 1000) / 100.00;
    ki = PotCalb * map(analogRead(potPinKi), 0, 1023, 0, 2000) / 1000.00;
    kd = PotCalb * map(analogRead(potPinKd), 0, 1023, 0, 2000) / 1000.00;
    digitalWrite(automaticLedPin, LOW);
    digitalWrite(manualLedPin, HIGH);
  }
  
  else{
    kp = autokp;
    ki = autoki;
    kd = autokd;
    digitalWrite(automaticLedPin, HIGH);
    digitalWrite(manualLedPin, LOW);
  }
  int target = int(map(analogRead(potPinTarget), 0, 1023, 0, 360));



  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  
  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT); //forward differance differantiation

  // integral
  eintegral = eintegral + e*deltaT; //Eular integration

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print("target: ");
  Serial.print(target);
  Serial.print(" Error: ");
  Serial.print(target-pos);
  Serial.print(" Mode: ");
  Serial.print(isAutomaticMode ? "Auto" : "Manual");
  Serial.print("Kp: ");
  Serial.print(isAutomaticMode ? autoKp : kp);
  Serial.print("  Ki: ");
  Serial.print(isAutomaticMode ? autoKi : ki);
  Serial.print("  Kd: ");
  Serial.println(isAutomaticMode ? autoKd : kd);
  if(millis()-lcdUpdateT>2000){
  lcdUpdateT = millis();
  lcd.clear();
  lcd.print("Mode: ");
  lcd.print(isAutomaticMode ? "Auto" : "Manual");
  lcd.setCursor(0,1);
  if(flag==0){
  lcd.print("Kp: ");
  lcd.print(kp);
  lcd.print(" Ki: ");
  lcd.print(ki);
  flag=1;
  }
  else{
  lcd.print("Kd: ");
  lcd.print(kd);
  flag=0;} 

  }

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
