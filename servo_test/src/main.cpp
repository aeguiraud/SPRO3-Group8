#define STEP_LENGTH 30 // Max length of a step in mm
#define LEG_LENGTH 50 // mm
#define SERVOMIN  240 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  410 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define PUT_DOWN 0
#define PROPELL 1
#define LIFT_UP 2

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


uint8_t servonum = 0;

uint8_t isright = 0; // Variable for prioritizing left or right side in a step (1 = true, 0 = false)
uint8_t step_state = 1; // 0,1,2,3

int timer_overflow = 0;

float speed = 5; // Walking speed in mm/s
float step_distance = 0; // Distance Gort. has traveled since the start of the step

// float pitch[6]; // Motor step_distances for pitch (up down shoulder motion)
// float yaw[6]; // Motor step_distances for yaw (forward and back)
// float bend[6]; // Motor step_distances for bending the knee

typedef struct{
	float yaw;
	float pitch;
	float bend;
}leg_t;

leg_t legs[6];



void calculate(void);
void actuate(void);


void setup(){
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
 
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
  
  
  TCCR2B |= (1 << CS22) | (1<< CS21) | (1 << CS20); // Set prescaler to 1024 and start Timer2
  TIMSK2 |= (1 << TOIE2); // Overflow interrupt enable

}

void loop(){
    // Motors are grouped by function, not leg
    // Serial.println(timer_overflow);

    // step_distance = 0;
    // while(step_distance <= STEP_LENGTH){
      float time_elapsed = (TCNT2 + timer_overflow*1024)*0.000064; // Load time elapsed since last calculation (accounting for overflow by addig the max value of Timer0 timer_overflow times)
      timer_overflow = 0;
      float distance_to_actuate = speed*time_elapsed; // Calculate distance to reach before next calculation (in mm)
      step_distance += distance_to_actuate; // Update step_distance

			calculate();

      if(step_distance >= STEP_LENGTH) {
				isright ^= 1; // Toggle side
        step_state++;
        step_distance = 0;
      }

      actuate();
  

      TCCR2B |= (1 << CS22) | (1<< CS21) | (1 << CS20); // Restart Timer0
    // }

}

void calculate(void){
  	switch (step_state%3){
        case PUT_DOWN:
          // yaw[0 + isright] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					// yaw[2 + isright] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Might receive different calculations later
					// yaw[4 + isright] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
          // pitch[0 + isright] = 2.5*yaw[0 + isright]; // random scaler
					// pitch[2 + isright] = 2.5*yaw[2 + isright];
					// pitch[4 + isright] = 2.5*yaw[4 + isright];

					legs[0 + isright].yaw = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					legs[2 + isright].yaw = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					legs[4 + isright].yaw = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					legs[0 + isright].pitch = 2.5*legs[0 + isright].yaw;
					legs[2 + isright].pitch = 2.5*legs[2 + isright].yaw;
					legs[4 + isright].pitch = 2.5*legs[4 + isright].yaw;
          break;
        
        case PROPELL:
          // yaw[0 + isright] = asin((step_distance - (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with negative values
					// yaw[2 + isright] = asin((step_distance - (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with negative values
					// yaw[4 + isright] = asin((step_distance - (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with negative values

					legs[0 + isright].yaw = asin((step_distance - (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with negative values
					legs[2 + isright].yaw = asin((step_distance - (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with negative values
					legs[4 + isright].yaw = asin((step_distance - (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with negative values
          break;

        case LIFT_UP:
          // yaw[0 + isright] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					// yaw[2 + isright] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					// yaw[4 + isright] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
          // pitch[0 + isright] = (-2.5)*yaw[0 + isright]; // random scaler
					// pitch[2 + isright] = (-2.5)*yaw[2 + isright];
					// pitch[4 + isright] = (-2.5)*yaw[4 + isright];

					legs[0 + isright].yaw = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					legs[2 + isright].yaw = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					legs[4 + isright].yaw = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
					legs[0 + isright].pitch = (-2.5)*legs[0 + isright].yaw; // random scaler
					legs[2 + isright].pitch = (-2.5)*legs[2 + isright].yaw; // random scaler
					legs[4 + isright].pitch = (-2.5)*legs[4 + isright].yaw; // random scaler
					
          if(step_distance >= STEP_LENGTH/2) step_state++;
          break;

        default:
          break;


		// float bend_offset = cos(yaw[0 + isright])*LEG_LENGTH - cos(0.2*LEG_LENGTH); // - sin(30°)*leg_length
    // bend[0 + isright] = 2*asin(bend_offset/LEG_LENGTH); // random scaler

		// bend_offset = cos(yaw[2 + isright])*LEG_LENGTH - cos(0.2*LEG_LENGTH); // - sin(30°)*leg_length
		// bend[2 + isright] = 2*asin(bend_offset/LEG_LENGTH); // random scaler

		// bend_offset = cos(yaw[4 + isright])*LEG_LENGTH - cos(0.2*LEG_LENGTH); // - sin(30°)*leg_length
		// bend[4 + isright] = 2*asin(bend_offset/LEG_LENGTH); // random scaler

		float bend_offset = cos(legs[0 + isright].yaw)*LEG_LENGTH - cos(0.2*LEG_LENGTH); // - sin(30°)*leg_length
    legs[0 + isright].bend = 2*asin(bend_offset/LEG_LENGTH); // random scaler

		bend_offset = cos(legs[2 + isright].yaw)*LEG_LENGTH - cos(0.2*LEG_LENGTH); // - sin(30°)*leg_length
    legs[2 + isright].bend = 2*asin(bend_offset/LEG_LENGTH); // random scaler

		bend_offset = cos(legs[2 + isright].yaw)*LEG_LENGTH - cos(0.2*LEG_LENGTH); // - sin(30°)*leg_length
    legs[2 + isright].bend = 2*asin(bend_offset/LEG_LENGTH); // random scaler
}
}

void actuate(void){

		for(int i=0; i<3; i++){
			// yaw[2*i + isright] = yaw[2*i + isright]*(SERVOMAX - SERVOMIN)/PI + 240;
			// pitch[2*i + isright] = pitch[2*i + isright]*(SERVOMAX - SERVOMIN)/PI + 300;
			// bend[2*i + isright] = bend[2*i + isright]*(SERVOMAX - SERVOMIN)/PI + 220;

			legs[2*i + isright].yaw = legs[2*i + isright].yaw*(SERVOMAX - SERVOMIN)/PI + 240;
			legs[2*i + isright].pitch = legs[2*i + isright].pitch*(SERVOMAX - SERVOMIN)/PI + 300;
			legs[2*i + isright].bend = legs[2*i + isright].bend*(SERVOMAX - SERVOMIN)/PI + 220;
			
			// if((yaw[2*i + isright] < SERVOMAX) && (yaw[2*i + isright] > SERVOMIN)) pwm.setPWM(0, 0, yaw[2*i + isright]);
			// if((pitch[2*i + isright] < SERVOMAX) && (pitch[2*i + isright] > SERVOMIN)) pwm.setPWM(1, 0, pitch[2*i + isright]);
			// if((bend[2*i + isright] < SERVOMAX) && (bend[2*i + isright] > SERVOMIN)) pwm.setPWM(2, 0, bend[2*i + isright]);

			if((legs[2*i + isright].yaw < SERVOMAX) && (legs[2*i + isright].yaw > SERVOMIN)) pwm.setPWM(0, 0, legs[2*i + isright].yaw);
			if((legs[2*i + isright].pitch < SERVOMAX) && (legs[2*i + isright].pitch > SERVOMIN)) pwm.setPWM(1, 0, legs[2*i + isright].pitch);
			if((legs[2*i + isright].bend < SERVOMAX) && (legs[2*i + isright].bend > SERVOMIN)) pwm.setPWM(2, 0, legs[2*i + isright].bend);
		}
}

ISR(TIMER2_OVF_vect){ // On Timer0 overflow
    timer_overflow++;
}