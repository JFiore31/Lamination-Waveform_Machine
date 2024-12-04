#include <Arduino.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>

//Define Pins
int spd = A0;
int enable = 2;
int dir = 3;
int driverEnable = 13;
int driverDir = 12;
int driverPul = 11;

//Define Vars
volatile bool setDir;
volatile bool setEnable;
volatile bool lastEnable;
volatile bool lastDir;
volatile long int pd;
String toDisplay;
int pulseType;
volatile int dirPin;
volatile int enablePin;
volatile int setSpd;
volatile bool LCDFlag = false;
volatile int timerCounter = 1;
volatile int rpm;

//Define LCD Board
LiquidCrystal_I2C lcd(0x27,16,2);

void setup() {
  Serial.begin(9600);
  //Serial.println("1");

  // Setup pins
  pinMode(driverDir, OUTPUT);
  pinMode(driverPul, OUTPUT);
  pinMode(driverEnable, OUTPUT);
  pinMode(dir, INPUT_PULLUP);
  pinMode(enable, INPUT_PULLUP);
  
  //Serial.println("2");
  //Setup the LCD displat to turn it on and be ready for commands
  lcd.init();
  lcd.backlight();
  //Serial.println("3");
  //////////////////////////////////////////////////////////////////////
  //Set up CPUTimer 4 so we can count when to update LCD and Dir/Enable
  //////////////////////////////////////////////////////////////////////
  cli();//stop interrupts
  //noInterrupts();
  //Serial.println("4");
  //set timer0 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for .5hz increments (2 sec)
  OCR1A = 31249;// = (16*10^6) / (.5*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12); 
  TCCR1B |= (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
  //Serial.println("5");
  //Serial.println("In Setup");
  //interrupts();
  //Serial.println("6");
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt .5Hz runs Dir/Enable functions and LCD updat
  dirPin = digitalRead(dir);
  enablePin = digitalRead(enable);
  if(timerCounter >= 4){
    LCDFlag = true;
    timerCounter = 0;
  }
  timerCounter++; //not sure how to access this repo, so made an int for it
}

void updateLCD(){
  ///////////////////////////////////////////////////////////////////
  //Display settings for LCD Display
  ///////////////////////////////////////////////////////////////////
  //Serial.println("updating LCD");
  //Map the period to a speed based on experimental RPM, not theory
  rpm = map(pd,2500,2.5,8,13);
  toDisplay = "RPM=" + String(rpm);
  lcd.setCursor(0,0);
  // if(rpm >= 1000){
  //   //print 4 digit RMP
  //   if(pulseType != 4){
  //     lcd.clear();
  //   }
  //   lcd.print(toDisplay);
  //   pulseType = 4;
  // } else if(rpm >= 100 && rpm < 1000){
  //   //print 3 digit RMP
  //   if(pulseType != 3){
  //     lcd.clear();
  //   }
  //   lcd.print(toDisplay);
  //   pulseType = 3;
  // } else 
  if(rpm > 9 && rpm <= 99){
    //print 2 digit RPM
    if(pulseType != 2){
      lcd.clear();
    }
    lcd.print(toDisplay);
    pulseType = 2;
  } else {
    //print 1 digit RPM
    if(pulseType != 1){
      lcd.clear();
    }
    lcd.print(toDisplay);
    pulseType = 1;
  }
  //Now print Direction and Enable setting
  lcd.setCursor(0,1);
  if(dirPin == LOW && enablePin == LOW) {
    lcd.print("Dir = CCW    OFF");
    //Serial.println("Low, low");
  } if (dirPin == HIGH && enablePin == LOW){
    lcd.print("Dir = CW     OFF");
    //Serial.println("High, low");
  } if (dirPin == HIGH && enablePin == HIGH){
    lcd.print("Dir = CW      ON");
    //Serial.println("high, high");
  } else if(dirPin == LOW && enablePin == HIGH) {
    lcd.print("Dir = CCW     ON");
    //Serial.println("Low, high");
  }
  LCDFlag = false;
}
void dirChange(){
  ///////////////////////////////////////////////////////////////////
  //Check for Rotation and Enable Change
  ///////////////////////////////////////////////////////////////////
  if(dirPin == LOW){
    //changes to CW
    setDir = LOW;
    lastDir = LOW;
    //Serial.println("setDir = LOW");
  } else if(dirPin == HIGH) {
    //Changes to CCW
    setDir = HIGH;
    lastDir = HIGH;
    //Serial.println("setDir = High");
  }
  digitalWrite(driverDir, setDir);
}
void enableChange(){
  if(enablePin == HIGH){
    //Turns motor on
    setEnable = HIGH;
    lastEnable = HIGH;
    //Serial.println("setEnable = LOW");
  } else if(enablePin == LOW){
    //turns motor off
    setEnable = LOW;
    lastEnable = LOW;
    //Serial.println("setEnable = High");

    //slow motor to a stop slowly
    pd = map(setSpd,0,1023,500,0.5);
    for(int i = pd; i <= 100; i++){
    digitalWrite(driverPul, HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPul, LOW);
    delayMicroseconds(pd);
    }
    //delayMicroseconds(500);

  }
  digitalWrite(driverEnable, setEnable);
}

void loop() {
  /////////////////////////////////////////////////////////////////////////
  //Logic to set direction and enable
  //Logic to send pd to stepper
  /////////////////////////////////////////////////////////////////////////
  //Serial.println("in loop");
  //Interrupts are not used for enable and dir changing, but they probably could (should) be
  //Instead, these if statement check based on the updates done in the CPU Timer 1 ISR
  if(enablePin != lastEnable){
      enableChange();
  }
  if(dirPin != lastDir) {
    dirChange();
  }
  // Logic to send PWM to motor based on potentiometer status
  setSpd = analogRead(spd);
  pd = map(setSpd,0,1023,500,0.5); //50:1 gear box used, so 50,.05 gets changed to 2500, 2.5
  digitalWrite(driverPul, HIGH);
  delayMicroseconds(pd);
  digitalWrite(driverPul, LOW);
  delayMicroseconds(pd);
  //Serial.println(updateLCDFlag);
  //Same as above if statements, should be an interrupt but this is a analog pin
  //This function takes a long time compared to PWM
  //if(LCDFlag){
  //updateLCD();
  //}
}
