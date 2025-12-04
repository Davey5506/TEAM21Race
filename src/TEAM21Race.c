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
volatile uint16_t speed[2] = {60, 65};
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

void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR13){
        start = !start;
        EXTI->PR |= EXTI_PR_PR13; // Clear pending bit
    }
}

int main(void){
    // Initialize SSD
    init_ssd(10);
    init_usart(115200);
    // Setup TIM3
    init_gp_timer(TIM3, TIM3_FREQ_HZ, PWM_PERIOD, false);
    // PWM mode 1 for CH3 and CH4
    TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) | (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable CH3 and CH4 outputs
    TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH; // PC8
    TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
    TIM3->CR1 |= TIM_CR1_CEN;

    init_gp_timer(TIM4, 1000, 30000, true);
    // Set up servos
    SERVO_t left_wheel = {
        .SERVO_PIN_PORT = GPIOC,
        .SERVO_PWM_PIN = 8,
        .SERVO_FEEDBACK_PIN = 16 // Does not exist, placeholder
    };
    SERVO_t right_wheel = {
        .SERVO_PIN_PORT = GPIOC,
        .SERVO_PWM_PIN = 9,
        .SERVO_FEEDBACK_PIN = 16 // Does not exist, placeholder
    };
    init_servo(&left_wheel);
    init_servo(&right_wheel);

    // Setup PMOD C for sensors
    init_pmod(PMOD_C);
    for(int i = 0; i < 4; i++){
        set_pin_mode(PMOD_C.PIN_PORTS[i], PMOD_C.PIN_NUMS[i], INPUT);
        set_pin_pull(PMOD_C.PIN_PORTS[i], PMOD_C.PIN_NUMS[i], PULL_DOWN);
    }

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR |= EXTI_IMR_IM13; 
    EXTI->FTSR |= EXTI_FTSR_TR13;
    set_pin_pull(GPIOC, 13, PULL_UP);
    set_pin_mode(GPIOC, 13, INPUT);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 2);

    while(1){
        read_uv_sensors();
        if(start){
            blank_drive();
        }else{
            TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
            TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
        }
    }
    return 0;
}