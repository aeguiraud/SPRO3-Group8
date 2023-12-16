#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  450 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// tested yaw limits: 150-400
// tested pitch limits: 130-250
// tested bend limits: 270-440

// yaw: -30°, 30°
// pitch: 20°, 0°
// bend: -20°, 15°

#include <stdio.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int value = 200;

void setup(){
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
 
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop(){
  for(int i=0; i*10 < SERVOMAX; i++){
    if((SERVOMIN + i*10 >= SERVOMIN) && (SERVOMIN + i*10 <= SERVOMAX)) pwm.setPWM(1, 0, SERVOMIN + i*10);
    Serial.print("pulselength: ");
    Serial.println(SERVOMIN + i*10);
    delay(500);
    // pwm.setPWM(3, 0, 340);
  }
  // for(int j=490; j > 370; j-=10){
  //   if((SERVOMIN + j >= SERVOMIN) && (SERVOMIN + j <= SERVOMAX)) pwm.setPWM(3, 0, SERVOMIN + j);
  //   Serial.print("pulselength: ");
  //   Serial.println(SERVOMIN + j);
  //   delay(1000);
  //   // pwm.setPWM(3, 0, 340);
  // }

  // pwm.setPWM(0, 0, 370);
  // pwm.setPWM(4, 0, 370);
  // delay(1000);
  
  
}