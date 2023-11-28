#define STEP_LENGTH 20 // Max length of a step in mm

// #include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  240 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  410 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PUT_DOWN 0
#define PROPELL 1
#define LIFT_UP 2
#define HOWER_BACK 3

// our servo # counter
uint8_t servonum = 0;

uint8_t left = 1; // Variable for prioritizing left or right side in a step (1 = true, 0 = false)
uint8_t step_state = 1; // 0,1,2,3

int timer_overflow = 0;

float speed = 10; // Walking speed in mm/s
float step_distance = 0; // Distance Gort. has traveled since the start of the step

float pitch[6]; // Motor positions for pitch (up down shoulder motion)
float yaw[6]; // Motor positions for yaw (forward and back)
float bend[6]; // Motor positions for bending the knee

void calculate(void);


int main(){
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
  
  
  TCCR2B |= (1 << CS22) | (1<< CS21) | (1 << CS20); // Set prescaler to 1024 and start Timer2
  TIMSK2 |= (1 << TOIE2); // Overflow interrupt enable

  while(1){
    // Motors are grouped by function, not leg
    Serial.print(timer_overflow);

    step_distance = 0;
    // while(step_distance <= STEP_LENGTH){
      float time_elapsed = TCNT2 + timer_overflow*1024*256/(16*10^6); // Load time elapsed since last calculation (accounting for overflow by addig the max value of Timer0 timer_overflow times)
      float distance_to_actuate = speed*time_elapsed; // Calculate distance to reach before next calculation (in mm)
      step_distance += distance_to_actuate; // Update step_distance


      switch (step_state%4){
        case PUT_DOWN:
          pitch[0] = (-0.5)*yaw[0];
          break;
        
        case PROPELL:
          yaw[0] = asin(step_distance - STEP_LENGTH/2); // Start with negative values
          break;

        case LIFT_UP:
          pitch[0] = 0.5*yaw[0];
          break;

        case HOWER_BACK:
          yaw[0] = asin((-step_distance) + STEP_LENGTH/2); // Start with positive values
          break;

        default:
          break;
      }

      float bend_offset = sin(yaw[0]) - 0.5*50; // - sin(30°)*leg_length
      bend[0] = asin(bend_offset);

      if(step_distance >= STEP_LENGTH) {
        step_state++;
        step_distance = 0;
      }

      map(yaw[0], (-PI/2), PI/2, 240, 410);
      map(pitch[0], (-PI/2), PI/2, 240, 410);
      map(bend[0], (-PI/2), PI/2, 240, 410);

      if((yaw[0] < SERVOMAX) && (yaw[0] > SERVOMIN)) pwm.setPWM(0, 0, yaw[0]);
      if((pitch[0] < SERVOMAX) && (pitch[0] > SERVOMIN)) pwm.setPWM(1, 0, pitch[0]);
      if((bend[0] < SERVOMAX) && (bend[0] > SERVOMIN)) pwm.setPWM(2, 0, bend[0]);
  

      TCCR2B |= (1 << CS22) | (1<< CS21) | (1 << CS20); // Restart Timer0
    // }

  }

}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
// void setServoPulse(uint8_t n, double pulse) {
//   double pulselength;
  
//   
// }

// void loop() {



  // Drive each servo one at a time using setPWM()
  // Serial.println(servonum);
  // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
  //   pwm.setPWM(servonum, 0, pulselen);
  //   delay(5);
  // }

  // delay(500);
  // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
  //   pwm.setPWM(servonum, 0, pulselen);
  //   delay(5);
  // }

  // delay(500);

  // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
  // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior. 
  // for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
  //   pwm.writeMicroseconds(servonum, microsec);
  // }

  // delay(500);
  // for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
  //   pwm.writeMicroseconds(servonum, microsec);
  // }

  // delay(500);

  // servonum++;
  // if (servonum > 2) servonum = 0; // Testing the first 8 servo channels
// }


// void calculate(void){

//     // float time_elapsed = TCNT0 + timer_overflow*1024*256/(16*10^6); // Load time elapsed since last calculation (accounting for overflow by addig the max value of Timer0 timer_overflow times)
//     // float distance_to_actuate = speed*time_elapsed; // Calculate distance to reach before next calculation (in mm)
//     // step_distance += distance_to_actuate; // Update step_distance

//     if(step_distance >= STEP_LENGTH) (left ^= 1); // Change side at the end of step

//     // TCCR0B |= (1 << CS00) | (1<< CS02); // Restart Timer0

//     // Trig calculations here
//     pitch[1 - left] = 0; // Depending on the side, the correct motors are automatically selected
//     pitch[3 - left] = 0;
//     pitch[5 - left] = 0;

//     yaw[1 - left] = 0;
//     yaw[3 - left] = 0;
//     yaw[5 - left] = 0;

//     bend[1 - left] = 0;
//     bend[3 - left] = 0;
//     bend[5 - left] = 0;

//     // timer_overflow = 0;
// }

ISR(TIMER2_OVF_vect){ // On Timer0 overflow
    timer_overflow++;
}