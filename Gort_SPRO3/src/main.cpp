#define STEP_LENGTH 20 // Max length of a step in mm

#include <stdio.h>
// #include <avr/io.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Arduino.h>

// Variable definitions
uint8_t left = 1; // Variable for prioritizing left or right side in a step (1 = true, 0 = false)

int direction; // Walking direction in degrees
int timer_overflow = 0;

float speed; // Walking speed in mm/s
float step_distance = 0; // Distance Gort. has traveled since the start of the step

float pitch[6]; // Motor positions for pitch (up down shoulder motion)
float yaw[6]; // Motor positions for yaw (forward and back)
float bend[6]; // Motor positions for bending the knee

// Function definitions
void calculate(void); // Function for calculating motor positions
void actuate(void); // Function for moving motors to the right positions

int main(){

    // TCCR0B |= (1 << CS00) | (1<< CS02); // Set prescaler to 1024 and start Timer0

    while(1){
        calculate();
        actuate();
    }

}

// Functions
void calculate(void){

    // float time_elapsed = TCNT0 + timer_overflow*1024*256/(16*10^6); // Load time elapsed since last calculation (accounting for overflow by addig the max value of Timer0 timer_overflow times)
    // float distance_to_actuate = speed*time_elapsed; // Calculate distance to reach before next calculation (in mm)
    // step_distance += distance_to_actuate; // Update step_distance

    if(step_distance >= STEP_LENGTH) (left ^= 1); // Change side at the end of step

    // TCCR0B |= (1 << CS00) | (1<< CS02); // Restart Timer0

    // Trig calculations here
    pitch[1 - left] = 0; 
    pitch[3 - left] = 0;
    pitch[5 - left] = 0;

    yaw[1 - left] = 0;
    yaw[3 - left] = 0;
    yaw[5 - left] = 0;

    bend[1 - left] = 0;
    bend[3 - left] = 0;
    bend[5 - left] = 0;

    timer_overflow = 0;
}

void actuate(void){

}

// ISR(TIMER0_OVF_vect){ // On Timer0 overflow
//     timer_overflow++;
// }