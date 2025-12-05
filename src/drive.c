#include "drive.h"
#include "hat.h"

static enum PIN_VALUE sensors[4] = {PIN_ERROR, PIN_ERROR, PIN_ERROR, PIN_ERROR};
static volatile uint16_t speed[2] = {115, 110};

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

void blank_drive(volatile uint8_t* mode){ 
    if(!sensors[0] && !sensors[1] && !sensors[2] && !sensors[3]){
        TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
        TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
        *mode = 1;
        TIM4->CR1 &= ~TIM_CR1_CEN; // Stop timer
    }else if(sensors[0] && sensors[1] && sensors[2] && sensors[3]){
        move_forward();
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

void read_uv_sensors(volatile uint16_t* sensor){
    *sensor = 0;
    for(int i = 3; i >= 0; i--){
        uint8_t val = !read_pin(PMOD_C.PIN_PORTS[i], PMOD_C.PIN_NUMS[i]);
        sensors[i] = val;
    }
    if(sensors[0]) sensor += 1000;
    if(sensors[1]) sensor += 100;
    if(sensors[2]) sensor += 10;
    if(sensors[3]) sensor += 1;
}