#define STEP_LENGTH 30 // Max length of a step in mm
#define FORELEG_LENGTH 60 // mm
#define THIGH_LENGTH 50 // mm
#define LEG_DISTANCE 70 // Vertical distance between separate legs in mm
#define DELTA 50 // Horizontal distanc between robot center and leg tips
#define SERVOMIN  240 // This is the 'minimum' pulse length count (out of 4096)
#define YAWMIN 340
#define YAWMAX 500
#define PITCHMIN 150
#define PITCHMAX 230
#define BENDMIN 170
#define BENDMAX	370
#define SERVOMAX  410 // This is the 'maximum' pulse length count (out of 4096)
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


uint8_t isright = 0; // Variable for prioritizing left or right side in a step (1 = true, 0 = false)
uint8_t L_step_state = PROPEL; // 0,1,2
uint8_t R_step_state = LIFT_UP; // 0,1,2
uint8_t neg; // Multiplication factor for turning
uint8_t turning = 1; // Whether Gort. is turning. 0 = not, -1 = left, 1 = right

int timer_overflow = 0;

float speed = 5; // Walking speed in mm/s
float step_distance = 0; // Distance Gort. has traveled since the start of the step
float R = 1000; // Turning radius in mm
float phi;

float pitch[6]; // Motor step_distances for pitch (up down shoulder motion)
float yaw[6]; // Motor step_distances for yaw (forward and back)
float bend[6]; // Motor step_distances for bending the knee
float leg_pos[6];


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
	float time_elapsed = (TCNT2 + timer_overflow*1024)*0.000064; // Load time elapsed since last calculation (accounting for overflow by addig the max value of Timer0 timer_overflow times)
	timer_overflow = 0;
	float distance_to_actuate = speed*time_elapsed; // Calculate distance to reach before next calculation (in mm)
	step_distance += distance_to_actuate; // Update step_distance

	if(step_distance >= STEP_LENGTH) {
		isright ^= 1; // Toggle side
		L_step_state++;
		R_step_state++;
		step_distance = 0;
	}

	calculate();
	actuate();

	TCCR2B |= (1 << CS22) | (1<< CS21) | (1 << CS20); // Restart Timer0
    // }

}

void calculate(void){
	for(int i=0; i<3; i++){
		leg_back(2*i + isright); // goes until 4 + isright
		propel(2*i + (1 - isright)); // goes until 4 + isright on every other loop
	}
	// if((step_distance >= STEP_LENGTH/2) && (L_step_state == LIFT_UP)) (L_step_state = PUT_DOWN);
	if(step_distance >= STEP_LENGTH/2){
		if(L_step_state == LIFT_UP) (L_step_state = PUT_DOWN);
		if(R_step_state == LIFT_UP) (R_step_state = PUT_DOWN);
	}

	Serial.print("L_step_state: ");
	Serial.print(L_step_state);
	Serial.print("     R_step_state: ");
	Serial.print(R_step_state);
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

	float bend_offset;

	phi = step_distance/R;
	
	if((turning < 0 && leg_number > 2) || (turning > 0 && leg_number < 3)) neg = 1;
	else (neg = -1);

	if(!turning){
		yaw[leg_number] = asin(((-step_distance) + (float)STEP_LENGTH/2)/(float)THIGH_LENGTH); // Start with positive values

		bend_offset = cos(yaw[leg_number])*FORELEG_LENGTH - 43.3013; // cos(yaw[leg_number])*LEG_LENGTH - cos(0.52)*LEG_LENGTH
		bend[leg_number] = asin(bend_offset/FORELEG_LENGTH);
	}

	else{ // Turning
	
		float alpha; // Leg-specific turning angle in radian
		float phi; // Turning angle in radian
		float r; // Leg-specific turning radius in mm
		float rho;
		float legs_x; //Leg-specific x-position from origin while turning
		float legs_y; //Leg-specific y-position from origin while turning

		alpha = atan(LEG_DISTANCE/(R + neg*DELTA));

		if(sin(alpha) != 0) (r = LEG_DISTANCE/sin(alpha));
		else (r = R + neg*DELTA);

		phi += alpha;

		legs_x = ((R + neg*DELTA)*cos(phi) - R);
		legs_y = (R + neg*DELTA)*sin(phi);

		yaw[leg_number] = atan((legs_y - leg_pos[leg_number])/(legs_x - leg_pos[leg_number]));
		rho = sqrt(pow((legs_x - leg_pos[leg_number]), 2) + pow((legs_y - leg_pos[leg_number]), 2));

		bend_offset = rho - FORELEG_LENGTH;
		bend[leg_number] = asin(bend_offset/FORELEG_LENGTH);
	}

	if((L_step_state == LIFT_UP) || (R_step_state == LIFT_UP)){ // lift up
		pitch[leg_number] = (-PITCH_SCALER)*yaw[leg_number]; // random scaler
	}
	else{ // put down
		pitch[leg_number] = PITCH_SCALER*yaw[leg_number];
	}
}

void propel(uint8_t leg_number){

	float bend_offset;

	phi = step_distance/R;

	if((turning < 0 && leg_number > 2) || (turning > 0 && leg_number < 3)) neg = 1;
	else (neg = -1);

	if(!turning){
		yaw[leg_number] = asin((step_distance - (float)STEP_LENGTH/2)/(float)THIGH_LENGTH); // Start with negative values

		float bend_offset = cos(yaw[leg_number])*FORELEG_LENGTH - 43.3013; // cos(yaw[leg_number])*LEG_LENGTH - cos(0.52)*LEG_LENGTH
		bend[leg_number] = asin(bend_offset/FORELEG_LENGTH); // random scaler
	}

	else{ // Turning

		float alpha; // Leg-specific turning angle in radian
		float phi; // Turning angle in radian
		float r; // Leg-specific turning radius in mm
		float rho;
		float legs_x; //Leg-specific x-position from origin while turning
		float legs_y; //Leg-specific y-position from origin while turning

		alpha = atan(LEG_DISTANCE/(R + neg*DELTA));

		if(sin(alpha) != 0) (r = LEG_DISTANCE/sin(alpha));
		else (r = R + neg*DELTA);

		phi += alpha;

		legs_x = ((R + neg*DELTA)*cos(phi) - R);
		legs_y = (R + neg*DELTA)*sin(phi);

		yaw[leg_number] = atan((legs_y - leg_pos[leg_number])/(legs_x - leg_pos[leg_number]));
		rho = sqrt(pow((legs_x - leg_pos[leg_number]), 2) + pow((legs_y - leg_pos[leg_number]), 2));

		bend_offset = rho - FORELEG_LENGTH;
		bend[leg_number] = asin(bend_offset/FORELEG_LENGTH);
	}
}

void actuate(void){

	for(int j=0; j<6; j++){
		yaw[j] = yaw[j]*(YAWMAX - YAWMIN)/(70*PI/180) + YAWMIN; // Range: -40°, 30°
		pitch[j] = pitch[j]*(PITCHMAX - PITCHMIN)/(40*PI/180) + PITCHMIN; // Range: -20°, 20°
		bend[j] = bend[j]*(BENDMAX - BENDMIN)/(85*PI/180) + BENDMIN; // Range: -60°, 25°
	}
	for(int k=0; k<5; k++){
		if((yaw[k] < YAWMAX) && (yaw[k] > YAWMIN)) pwm.setPWM(3*k, 0, yaw[k]);
		if((pitch[k] < PITCHMAX) && (pitch[k] > PITCHMIN)) pwm.setPWM(3*k+1, 0, pitch[k]);
		if((bend[k] < BENDMAX) && (bend[k] > BENDMIN)) pwm.setPWM(3*k+2, 0, bend[k]);
	}
  
}

ISR(TIMER2_OVF_vect){ // On Timer0 overflow
    timer_overflow++;
}