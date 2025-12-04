#include "hat.h"
#include <stdio.h>

#define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width for neutral position

#define CW_MAX_PULSE 1480 
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720
#define TIM3_FREQ_HZ 1000000
#define PWM_FREQ_HZ 50
#define PWM_PERIOD (TIM3_FREQ_HZ / PWM_FREQ_HZ) // 20000 ticks for 20ms period
#define STOP_SPEED 150
enum PIN_VALUE sensors[4] = {PIN_ERROR, PIN_ERROR, PIN_ERROR, PIN_ERROR};
enum PIN_VALUE prev_sensors[4] = {PIN_ERROR, PIN_ERROR, PIN_ERROR, PIN_ERROR};
volatile uint16_t speed[2] = {110, 110};
volatile uint8_t stop_lines = 0;
volatile uint16_t sensor = 0;
volatile bool start = false;

void move_forward(void){
    TIM3->CCR3 = CCW_MAX_PULSE - speed[0];
    TIM3->CCR4 = CW_MIN_PULSE + speed[1]; 
}

void turn_right(void){
    TIM3->CCR3 = CCW_MIN_PULSE + 40; 
    TIM3->CCR4 = CCW_MIN_PULSE + 10; 
}

void turn_left(void){
    TIM3->CCR3 = CW_MAX_PULSE - 30; 
    TIM3->CCR4 = CW_MAX_PULSE - 50; 
}

void blank_drive(void){ 
    if(!sensors[0] && !sensors[1] && !sensors[2] && !sensors[3]){
        TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
        TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
    }else if(sensors[1] && sensors[2]){
        if(sensors[0]){
            turn_left();
        }else if(sensors[3]){
            turn_right();
        }else{
            move_forward();
        }
    }else if(sensors[0] && sensors[1]){
        turn_left();
    }else if(sensors[2] && sensors[3]){
        turn_right();
    }
}

void read_uv_sensors(void){
    sensor = 0;
    for(int i = 3; i >= 0; i--){
        uint8_t val = !read_pin(PMOD_C.PIN_PORTS[i], PMOD_C.PIN_NUMS[i]);
        sensors[i] = val;
    }
    if(sensors[0]) sensor += 1000;
    if(sensors[1]) sensor += 100;
    if(sensors[2]) sensor += 10;
    if(sensors[3]) sensor += 1;

    display_num(sensor, 0);
}

int main(void){
    return 0;
}