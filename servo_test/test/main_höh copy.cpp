#define STEP_LENGTH 50 // Max length of a step in mm
#define RAVE_LENGTH 30 // Max length of a rave in mm
#define FORELEG_LENGTH 60 // Vertical distance from end of thigh to leg tip
#define THIGH_LENGTH 65 // Horizontal distance from beginning of leg to end of thigh
#define LEG_DISTANCE 70 // Vertical distance between separate legs in mm
#define DELTA 115 // Horizontal distance between robot center and leg tips
#define X_OFFSET 50 // Horizontal distance between robot center and yaw joint
#define YAWMIN 150 // Minimum pulse length for the yaw servos
#define YAWMAX 400 // Maximum pulse length for the yaw servos
#define PITCHMIN 200 // Minimum pulse length for the pitch servos
#define PITCHMAX 250 // Maximum pulse length for the pitch servos
#define BENDMIN 250 // Minimum pulse length for the bend servos
#define BENDMAX	420 // Maximum pulse length for the bend servos
#define PITCH_STEP (PITCHMAX-PITCHMIN) / ((STEP_LENGTH)/2) // Constant for pitch calculations while stepping
#define PITCH_RAVE (PITCHMAX-PITCHMIN) / ((RAVE_LENGTH)/2) // Constant for pitch calculations while raving
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PITCHSTART_LEFT 200
#define PITCHSTART_RIGHT 250

#define PUT_DOWN 0 // Leg is being pitched down to the beginning of a step
#define PROPEL 1 // Leg is moving along a line on the ground to move forward
#define LIFT_UP 2 // Leg is being pitched up to the half point of beginning of a step

// #include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Adafruit_PWMServoDriver.h>

// Called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


uint8_t isright = 0; // Variable for prioritizing left or right side in a step (1 = right, 0 = left)
uint8_t L_step_state = PROPEL; // 0,1,2
uint8_t R_step_state = LIFT_UP; // 0,1,2
uint8_t raving = 0; // Variable for whether Gort. is raving or not. 0 = not raving, 1 = raving
uint8_t timer_overflow = 0;

int neg; // Multiplication factor for turning
int side; // Multiplication factor for inverting legs. -1 = right, 1 = left
int turning = -1; // Which direction Gort. is turning. 1 = left, -1 = right
int forward = 1; // Whether Gort. is moving forwards (1) or backwards (-1)

float speed = 20; // Walking speed in mm/s
float step_distance = 0; // Distance Gort. has traveled since the start of the step
float rave_distance = 0; // Distance Gort. has raved since the start of the rave
float R = 2000000; // Turning radius in mm. Extremely large when walking straight
float phi; // Angle between beginning and end of the body during a step

float pitch[6]; // Motor step_distances for pitch (up down shoulder motion) in pulse length
float yaw[6]; // Motor step_distances for yaw (forward and back) in radian
float bend[6]; // Motor step_distances for bending the knee in radian

int leg_pos_x[2] = {-X_OFFSET, X_OFFSET}; // Leg x-positions relative to the center of the robot. Legs 0,2,4 and 1,3,5 respectively
int leg_pos_y[6] = {LEG_DISTANCE, 0, -LEG_DISTANCE}; // Leg y-positions relative to the center of the robot. Legs 0,3 and 1,4 and 2,5 respectively

void calculate(void);
void actuate(void);
void rave(void);

void leg_back(uint8_t);
void propel(uint8_t);


void setup(){
	Serial.begin(9600);
	// Serial.println("8 channel Servo test!");
	Serial.begin(9600);
	// Serial.println("8 channel Servo test!");

	pwm.begin();

	pwm.setOscillatorFrequency(27000000);
	pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

	delay(10);


	TCCR2B |= (1 << CS22) | (1<< CS21) | (1 << CS20); // Set prescaler to 1024 and start Timer2
	TIMSK2 |= (1 << TOIE2); // Overflow interrupt enable
	speed *= forward; 
	if(raving) speed = abs(speed);
	if(speed > 0) step_distance = 0;
	else step_distance = STEP_LENGTH;

	if(!raving){
		for(int m=0;m<3;m++) pitch[m] = PITCHSTART_LEFT;
		for(int n=3;n<6;n++) pitch[n] = PITCHSTART_RIGHT;
	}
}

void loop(){
	// Motors are grouped by function, not leg
	float time_elapsed = (TCNT2 + timer_overflow*1024)*0.000064; // Load time elapsed since last calculation (accounting for overflow by addig the max value of Timer0 timer_overflow times)
	timer_overflow = 0;
	float distance_to_actuate = speed*time_elapsed; // Calculate distance to reach before next calculation (in mm)
	step_distance += distance_to_actuate; // Update step_distance
	rave_distance += distance_to_actuate;
	// Serial.print("stepdist: ");
	// Serial.print(step_distance);

	if((step_distance) >= STEP_LENGTH || step_distance < 0) {
		isright ^= 1; // Toggle side
		L_step_state++;
		R_step_state++;
		if(speed > 0) step_distance = 0;
		else step_distance = STEP_LENGTH;
		
		if(!raving){
			for(int m=0;m<3;m++) pitch[m] = PITCHSTART_LEFT;
			for(int n=3;n<6;n++) pitch[n] = PITCHSTART_RIGHT;
		}
	}
	if(rave_distance >= RAVE_LENGTH) rave_distance = 0;

	if(!raving){
		calculate();
		actuate();
	}
	else{
		rave();
		actuate();
	}
	TCCR2B |= (1 << CS22) | (1<< CS21) | (1 << CS20); // Restart Timer0
}

void calculate(void){
	for(int i=0; i<3; i++){
		propel(2*i + isright); // Goes until 4 + isright on every other loop
		leg_back(2*i + (1 - isright)); // Goes until 4 + isright
		
	}
	// if((step_distance >= STEP_LENGTH/2) && (L_step_state == LIFT_UP)) (L_step_state = PUT_DOWN);
	if((abs(step_distance)) >= STEP_LENGTH/2){
		if(L_step_state == LIFT_UP) (L_step_state = PUT_DOWN);
		if(R_step_state == LIFT_UP) (R_step_state = PUT_DOWN);
	}

	// Serial.print("L_step_state: ");
	// Serial.print(L_step_state);
	// Serial.print("     R_step_state: ");
	// Serial.print(R_step_state);
	// Serial.print("     step_distance: ");
	// Serial.print(step_distance);
	// Serial.print("		yaw[0]: ");
	// Serial.print(yaw[0]);
	// Serial.print("     bend[0]: ");
	// Serial.print(bend[0]);
	// Serial.print("		pitch[0]: ");
	// Serial.print(pitch[0]);
	// Serial.print("		pitch[3]: ");
	// Serial.println(pitch[3]);

	
	// Serial.print("     yaw[1]: ");
	// Serial.print(yaw[1]);
	// Serial.print("     yaw[2]: ");
	// Serial.print(yaw[2]);
	// Serial.print("     yaw[3]: ");
	// Serial.print(yaw[3]);
	// Serial.print("     yaw[4]: ");
	// Serial.print(yaw[4]);
	// Serial.print("     yaw[5]: ");
	// Serial.println(yaw[5]);

}

void propel(uint8_t leg_number){

	phi = -(step_distance - (float)STEP_LENGTH/2)/R;
	// Serial.print("phi: ");
	// Serial.print(phi);

	// //// Serial.print("     Max's thing: ");
	// //// Serial.print(step_distance - STEP_LENGTH);

	if((turning < 0 && leg_number < 3) || (turning > 0 && leg_number > 2)) neg = 1;
	else (neg = -1);
	//// Serial.print("	neg: ");
	//// Serial.print(neg);

	float alpha; // Leg-specific turning angle in radian|
	float r;
	float rho;
	float leg_x; //Leg-specific x-position from origin while turning
	float leg_y; //Leg-specific y-position from origin while turning

	alpha = (atan(leg_pos_y[leg_number%3]/(R + neg*DELTA)));
	// Serial.print("	alpha: ");
	// Serial.print(alpha);

	phi += alpha;
	// Serial.print("	phi2: ");
	// Serial.print(phi);

	if(sin(alpha)) r = abs(LEG_DISTANCE/sin(alpha));
	else r = R + neg*DELTA;
	// Serial.print("	r: ");
	// Serial.print(r);

	leg_x = turning*(r*cos(phi) - R*cos(phi));
	// Serial.print("	leg_x: ");
	// Serial.print(leg_x);
	leg_y = r*sin(phi);
	// Serial.print("	leg_y: ");
	// Serial.print(leg_y);

	if(leg_number/3) side = -1;
	else side = 1;

	yaw[leg_number] = (atan((leg_y - leg_pos_y[leg_number%3])/(leg_x - leg_pos_x[leg_number/3])));
	// Serial.print("	yaw[0]: ");
	// Serial.print(yaw[leg_number]);
	rho = sqrt(pow((leg_x - leg_pos_x[leg_number/3]), 2) + pow((leg_y - leg_pos_y[leg_number%3]), 2));
	// Serial.print("	rho: ");
	// Serial.println(rho);

	bend[leg_number] = asin((rho - FORELEG_LENGTH)/FORELEG_LENGTH);
	// //// Serial.print("	bend[0]: ");
	// //// Serial.println(bend[leg_number]);

}

void leg_back(uint8_t leg_number){

	phi = (step_distance - (float)STEP_LENGTH/2)/R;
	// Serial.print("phi: ");
	// Serial.print(phi);

	// //// Serial.print("     Max's thing: ");
	// //// Serial.print(step_distance - STEP_LENGTH);

	if((turning < 0 && leg_number < 3) || (turning > 0 && leg_number > 2)) neg = 1;
	else (neg = -1);
	//// Serial.print("	neg: ");
	//// Serial.print(neg);

	float alpha; // Leg-specific turning angle in radian|
	float r;
	float leg_x; //Leg-specific x-position from origin while turning
	float leg_y; //Leg-specific y-position from origin while turning

	alpha = (atan(leg_pos_y[leg_number%3]/(R + neg*DELTA)));
	// Serial.print("	alpha: ");
	// Serial.print(alpha);

	phi += alpha;
	// Serial.print("	phi2: ");
	// Serial.print(phi);

	if(sin(alpha)) r = abs(LEG_DISTANCE/sin(alpha));
	else r = R + neg*DELTA;
	// Serial.print("	r: ");
	// Serial.print(r);

	leg_x = turning*(r*cos(phi) - R*cos(phi));
	// Serial.print("	leg_x: ");
	// Serial.print(leg_x);
	leg_y = r*sin(phi);
	// Serial.print("	leg_y: ");
	// Serial.print(leg_y);

	if(leg_number/3) side = -1;
	else side = 1;

	yaw[leg_number] = (atan((leg_y - leg_pos_y[leg_number%3])/(leg_x - leg_pos_x[leg_number/3])));
	// Serial.print("	yaw[0]: ");
	// Serial.print(yaw[leg_number]);

	if((((L_step_state == LIFT_UP) || (L_step_state == PUT_DOWN)) && !(leg_number%2)) || (((R_step_state == LIFT_UP) || (R_step_state == PUT_DOWN)) && (leg_number%2))){ // Lift up
		if(side == 1) pitch[leg_number] = PITCHMAX - (abs(step_distance - (float)STEP_LENGTH/2) * PITCH_STEP);
		else pitch[leg_number] = PITCHMIN + (abs(step_distance - (float)STEP_LENGTH/2) * PITCH_STEP);
	}
}

void actuate(void){
	uint8_t shift = 0;
	for(int j=0; j<6; j++){
		if(!raving) yaw[j] = yaw[j]*(YAWMAX - YAWMIN)/(60*PI/180) + (float)(YAWMAX+YAWMIN)/2; // Range: -40°, 30°
		// Serial.print("yaw[3]: ");
		// Serial.print(yaw[3]);
		// Serial.print("		R_step_state: ");
		// Serial.println(R_step_state);
		bend[j] = bend[j]*(BENDMAX - BENDMIN)/(35*PI/180) + (float)(BENDMIN+BENDMAX)/2; // Range: -60°, 25°

		if(!raving) if((yaw[j] < YAWMAX) && (yaw[j] > YAWMIN)) pwm.setPWM((3*j)-shift, 0, yaw[j]);
		if((pitch[j] < PITCHMAX) && (pitch[j] > PITCHMIN)){
			if(j == 2){
				pwm.setPWM(1, 0, pitch[j]);
				shift++;
			}
			else if(j == 5){
				pwm.setPWM(9, 0, pitch[j]);
				shift++;
			}
			else pwm.setPWM((3*j+1)-shift, 0, pitch[j]);
		}
		if((bend[j] < BENDMAX) && (bend[j] > BENDMIN)) pwm.setPWM((3*j+2)-shift, 0, bend[j]);
		
	}
}

void rave(){
	bend[0] = bend[1] = bend[2] = asin((rave_distance - RAVE_LENGTH/2)/FORELEG_LENGTH), bend[3] = bend[4] = bend[5] = -bend[0];
	pitch[0] = pitch[1] = pitch[2] = pitch[3] = pitch[4] = pitch[5] = PITCHMAX - (abs(rave_distance - (float)RAVE_LENGTH/2) * PITCH_RAVE);

	// Serial.print("pitch[0]: ");
	// Serial.print(pitch[0]);
	// Serial.print("	pitch[1]: ");
	// Serial.print(pitch[1]);
	// Serial.print("	pitch[2]: ");
	// Serial.print(pitch[2]);
	// Serial.print("	pitch[3]: ");
	// Serial.print(pitch[3]);
	// Serial.print("	pitch[4]: ");
	// Serial.print(pitch[4]);
	// Serial.print("	pitch[5]: ");
	// Serial.print(pitch[5]);

	// Serial.print("bend[0]: ");
	// Serial.print(bend[0]);
	// Serial.print("	bend[1]: ");
	// Serial.print(bend[1]);
	// Serial.print("	bend[2]: ");
	// Serial.print(bend[2]);
	// Serial.print("	bend[3]: ");
	// Serial.print(bend[3]);
	// Serial.print("	bend[4]: ");
	// Serial.print(bend[4]);
	// Serial.print("	bend[5]: ");
	// Serial.print(bend[5]);
	// Serial.print("	ravedist: ");
	// Serial.println(rave_distance);
}


ISR(TIMER2_OVF_vect){ // On Timer0 overflow
    timer_overflow++;
}

// ISR(USART_RX_vect){
// 	volatile int received_data = UDR0;

// }