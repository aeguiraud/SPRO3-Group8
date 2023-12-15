#define STEP_LENGTH 50 // Max length of a step in mm
#define FORELEG_LENGTH 60 // mm
#define THIGH_LENGTH 65 // mm
#define LEG_DISTANCE 70 // Vertical distance between separate legs in mm
#define DELTA 115 // Horizontal distanc between robot center and leg tips
#define X_OFFSET 50 // Horizontal distanc between robot center and yaw joint
#define SERVOMIN  240 // This is the 'minimum' pulse length count (out of 4096)
#define YAWMIN 370
#define YAWMAX 490
#define PITCHMIN 200
#define PITCHMAX 250
#define BENDMIN 230
#define BENDMAX	350
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

// Called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


uint8_t isright = 1; // Variable for prioritizing left or right side in a step (1 = true, 0 = false)
uint8_t L_step_state = PROPEL; // 0,1,2
uint8_t R_step_state = LIFT_UP; // 0,1,2

int timer_overflow = 0;
int neg; // Multiplication factor for turning
int turning = 1; // Whether Gort. is turning. 0 = not, 1 = left, -1 = right

float speed = 15; // Walking speed in mm/s
float step_distance = 0; // Distance Gort. has traveled since the start of the step
float R = 1000; // Turning radius in mm
float phi;

float pitch[6]; // Motor step_distances for pitch (up down shoulder motion)
float yaw[6]; // Motor step_distances for yaw (forward and back)
float bend[6]; // Motor step_distances for bending the knee

int leg_pos_y[6] = {LEG_DISTANCE, 0, -LEG_DISTANCE};

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
		propel(2*i + (1 - isright)); // Goes until 4 + isright on every other loop
		leg_back(2*i + isright); // Goes until 4 + isright
		
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
	// Serial.print("     step_distance: ");
	// Serial.print(step_distance);
	Serial.print("		yaw[0]: ");
	Serial.print(yaw[0]);
	// Serial.print("     pitch[0]: ");
	// Serial.print(pitch[0]);
	// Serial.print("     bend[0]: ");
	// Serial.print(bend[0]);
	// Serial.print("     pitch[1]: ");
	// Serial.println(pitch[1]);

	
	Serial.print("     yaw[1]: ");
	Serial.print(yaw[1]);
	Serial.print("     yaw[2]: ");
	Serial.print(yaw[2]);
	Serial.print("     yaw[3]: ");
	Serial.print(yaw[3]);
	Serial.print("     yaw[4]: ");
	Serial.print(yaw[4]);
	Serial.print("     yaw[5]: ");
	Serial.println(yaw[5]);
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

	// if(!turning){
	// 	yaw[leg_number] = atan((step_distance - (float)STEP_LENGTH/2)/(float)THIGH_LENGTH); // Start with negative values

	// 	float bend_offset = cos(yaw[leg_number])*FORELEG_LENGTH - 43.3013; // cos(yaw[leg_number])*LEG_LENGTH - cos(0.52)*LEG_LENGTH
	// 	bend[leg_number] = asin(bend_offset/FORELEG_LENGTH); // Random scaler
	// }

	float alpha; // Leg-specific turning angle in radian|
	float r;
	float rho;
	float leg_x; //Leg-specific x-position from origin while turning
	float leg_y; //Leg-specific y-position from origin while turning

	int leg_pos_x[2] = {-X_OFFSET, X_OFFSET};

	alpha = atan(leg_pos_y[leg_number%3]/(R + neg*DELTA));
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

	yaw[leg_number] = atan((leg_y - leg_pos_y[leg_number%3])/(leg_x - leg_pos_x[leg_number/3]));
	// Serial.print("	yaw[0]: ");
	// Serial.print(yaw[leg_number]);
	// yaw[leg_number] = atan((leg_y - 0)/(leg_x - (-DELTA)));
	rho = sqrt(pow((leg_x - leg_pos_x[leg_number/3]), 2) + pow((leg_y - leg_pos_y[leg_number%3]), 2));
	// Serial.print("	rho: ");
	// Serial.println(rho);

	bend[leg_number] = asin((rho - FORELEG_LENGTH)/FORELEG_LENGTH);
	// //// Serial.print("	bend[0]: ");
	// //// Serial.println(bend[leg_number]);

}

void leg_back(uint8_t leg_number){

	phi = (step_distance - (float)STEP_LENGTH/2)/R;
	//// Serial.print("phi: ");
	//// Serial.print(phi);

	// //// Serial.print("     Max's thing: ");
	// //// Serial.print(step_distance - STEP_LENGTH);

	if((turning < 0 && leg_number < 3) || (turning > 0 && leg_number > 2)) neg = 1;
	else (neg = -1);
	//// Serial.print("	neg: ");
	//// Serial.print(neg);

	// if(!turning){
	// 	yaw[leg_number] = atan((step_distance - (float)STEP_LENGTH/2)/(float)THIGH_LENGTH); // Start with negative values

	// 	float bend_offset = cos(yaw[leg_number])*FORELEG_LENGTH - 43.3013; // cos(yaw[leg_number])*LEG_LENGTH - cos(0.52)*LEG_LENGTH
	// 	bend[leg_number] = asin(bend_offset/FORELEG_LENGTH); // Random scaler
	// }

	float alpha; // Leg-specific turning angle in radian|
	float r;
	float rho;
	float leg_x; //Leg-specific x-position from origin while turning
	float leg_y; //Leg-specific y-position from origin while turning

	int leg_pos_x[2] = {turning*X_OFFSET, -turning*X_OFFSET};

	alpha = atan(leg_pos_y[leg_number%3]/(R + neg*DELTA));
	//// Serial.print("	alpha: ");
	//// Serial.print(alpha);

	phi += alpha;
	//// Serial.print("	phi2: ");
	//// Serial.print(phi);

	if(sin(alpha)) r = abs(LEG_DISTANCE/sin(alpha));
	else r = R + neg*DELTA;
	//// Serial.print("	r: ");
	//// Serial.print(r);

	leg_x = turning*(r*cos(phi) - R*cos(phi));
	//// Serial.print("	leg_x: ");
	//// Serial.print(leg_x);
	leg_y = r*sin(phi);
	//// Serial.print("	leg_y: ");
	//// Serial.print(leg_y);

	yaw[leg_number] = atan((leg_y - leg_pos_y[leg_number%3])/(leg_x - leg_pos_x[leg_number/3]));
	//// Serial.print("	yaw[0]: ");
	//// Serial.print(yaw[leg_number]);
	// yaw[leg_number] = atan((leg_y - 0)/(leg_x - (-DELTA)));
	rho = sqrt(pow((leg_x - leg_pos_x[leg_number/3]), 2) + pow((leg_y - leg_pos_y[leg_number%3]), 2));
	//// Serial.print("	rho: ");
	//// Serial.println(rho);

	bend[leg_number] = asin((rho - FORELEG_LENGTH)/FORELEG_LENGTH);
	// Serial.print("	bend[0]: ");
	// Serial.println(bend[leg_number]);

	if(((L_step_state == LIFT_UP) && !(leg_number%2)) || ((R_step_state == LIFT_UP) && (leg_number%2))){ // Lift up
		pitch[leg_number] = (-(float)PITCH_SCALER)*yaw[leg_number]; // Random scaler
	}
	else pitch[leg_number] = (float)PITCH_SCALER*yaw[leg_number]; // Put down
}

void actuate(void){
	float pitch_pulse[6];

	for(int j=0; j<6; j++){
		yaw[j] = yaw[j]*(YAWMAX - YAWMIN)/(60*PI/180) + (float)(YAWMAX+YAWMIN)/2; // Range: -40°, 30°
		pitch_pulse[j] = pitch[j]*(PITCHMAX - PITCHMIN)/((0-20)*PI/180) + PITCHMIN;
		bend[j] = bend[j]*(BENDMAX - BENDMIN)/((15+20)*PI/180) + BENDMIN; // Range: -60°, 25°
	}
	for(int k=0; k<5; k++){
		if((yaw[k] < YAWMAX) && (yaw[k] > YAWMIN)) pwm.setPWM(3*k, 0, yaw[k]);
		if((pitch_pulse[k] < PITCHMAX) && (pitch_pulse[k] > PITCHMIN)) pwm.setPWM(3*k+1, 0, pitch_pulse[k]);
		if((bend[k] < BENDMAX) && (bend[k] > BENDMIN)) pwm.setPWM(3*k+2, 0, bend[k]);
	}
  
}


ISR(TIMER2_OVF_vect){ // On Timer0 overflow
    timer_overflow++;
}