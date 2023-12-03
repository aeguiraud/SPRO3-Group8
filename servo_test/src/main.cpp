#define STEP_LENGTH 30 // Max length of a step in mm
#define LEG_LENGTH 50 // mm
#define SERVOMIN  240 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  410 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PITCH_SCALER 1.5

#define PUT_DOWN 0
#define PROPEL 1
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

float pitch[6]; // Motor step_distances for pitch (up down shoulder motion)
float yaw[6]; // Motor step_distances for yaw (forward and back)
float bend[6]; // Motor step_distances for bending the knee

// typedef struct{
// 	float yaw;
// 	float pitch;
// 	float bend;
// }leg_t;

// leg_t legs[6];


void calculate(void);
void actuate(void);

void leg_back(uint8_t);
void propel(uint8_t);


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
	for(int i=0; i<3; i++){
		leg_back(2*i + isright); // goes until 4 + isright
		propel(2*i + (1 - isright)); // goes until 4 + isright on every other loop
	}
	if((step_distance >= STEP_LENGTH/2) && step_state == 2) step_state++;

	Serial.print("step_state: ");
	Serial.print(step_state%3);
	Serial.print("     step_distance: ");
	Serial.print(step_distance);
	Serial.print("     yaw[0]: ");
	Serial.print(yaw[0]);
	Serial.print("     pitch[0]: ");
	Serial.print(pitch[0]);
	Serial.print("     bend[0]: ");
	Serial.print(bend[0]);
	Serial.print("     pitch[1]: ");
	Serial.println(pitch[1]);
}

void leg_back(uint8_t leg_number){

	if((step_state%3) == 2){ // lift up
		yaw[leg_number] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
		pitch[leg_number] = (-PITCH_SCALER)*yaw[leg_number]; // random scaler
		// pitch[leg_number] = yaw[leg_number];
	}
	else{ // put down
		yaw[leg_number] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with positive values
		pitch[leg_number] = PITCH_SCALER*yaw[leg_number];
		// pitch[leg_number] = yaw[leg_number];
	}

	float bend_offset = cos(yaw[leg_number])*LEG_LENGTH - cos(0.2)*LEG_LENGTH; // - sin(30°)*leg_length
    bend[leg_number] = 2*asin(bend_offset/LEG_LENGTH); // random scaler
}

void propel(uint8_t leg_number){
	yaw[leg_number] = asin((step_distance - (float)STEP_LENGTH/2)/(float)LEG_LENGTH); // Start with negative values

	float bend_offset = cos(yaw[leg_number])*LEG_LENGTH - cos(0.2)*LEG_LENGTH; // - sin(30°)*leg_length
    bend[leg_number] = 2*asin(bend_offset/LEG_LENGTH); // random scaler
}

void actuate(void){

	for(int j=0; j<6; j++){
		yaw[j] = yaw[j]*(SERVOMAX - SERVOMIN)/PI + 240;
		pitch[j] = pitch[j]*(SERVOMAX - SERVOMIN)/PI + 300;
		bend[j] = bend[j]*(SERVOMAX - SERVOMIN)/PI + 220;
	}
	for(int k=0; k<5; k++){
		if((yaw[k] < SERVOMAX) && (yaw[k] > SERVOMIN)) pwm.setPWM(3*k, 0, yaw[k]);
		if((pitch[k] < SERVOMAX) && (pitch[k] > SERVOMIN)) pwm.setPWM(3*k+1, 0, pitch[k]);
		if((bend[k] < SERVOMAX) && (bend[k] > SERVOMIN)) pwm.setPWM(3*k+2, 0, bend[k]);
	}
  
}

ISR(TIMER2_OVF_vect){ // On Timer0 overflow
    timer_overflow++;
}