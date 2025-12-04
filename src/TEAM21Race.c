#include "hat.h"
#include <stdio.h>

#define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width for neutral position

// Drive Servos
#define CW_MAX_PULSE 1480 
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720
#define TIM3_FREQ_HZ 1000000
#define PWM_FREQ_HZ 50
#define PWM_PERIOD (TIM3_FREQ_HZ / PWM_FREQ_HZ) // 20000 ticks for 20ms period
#define STOP_SPEED 150

// Ultrasonic Servo
#define SENSOR_SERVO_CH TIM8->CCR3
#define SERVO_LEFT 1280
#define SERVO_CENTER 1500
#define SERVO_RIGHT 1720

enum PIN_VALUE sensors[4] = {PIN_ERROR, PIN_ERROR, PIN_ERROR, PIN_ERROR};
volatile uint16_t speed[2] = {115, 110};
volatile uint8_t mode = 0;
volatile uint16_t sensor = 0;
volatile bool start = false;

uint32_t ultrasonic_measure(void){
    uint32_t count=0;

    write_pin(ULTRA_SOUND.TRIG_PORT,ULTRA_SOUND.TRIG_PIN,1);
    delay_us(10);
    write_pin(ULTRA_SOUND.TRIG_PORT,ULTRA_SOUND.TRIG_PIN,0);

    while(!read_pin(ULTRA_SOUND.ECHO_PORT,ULTRA_SOUND.ECHO_PIN));

    while(read_pin(ULTRA_SOUND.ECHO_PORT,ULTRA_SOUND.ECHO_PIN)){
        count++;
    }
    return count;
}

void sensor_left(void){
    SENSOR_SERVO_CH= SERVO_LEFT;
    delay_us(300);
}
void sensor_center(void){
    SENSOR_SERVO_CH= SERVO_CENTER;
    delay_us(300);
}
void sensor_right(void){
    SENSOR_SERVO_CH=SERVO_RIGHT;
    delay_us(300);
}

int direction(void){
    uint32_t left,right;

    //left
    sensor_left();
    left=ultrasonic_measure();

    //back to center
    sensor_center();

    //right
    sensor_right();
    right=ultrasonic_measure();

    sensor_center();

    return (left > right) ? 0:1;  //0 is left, 1 is right
}  


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
        TIM4->CR1 &= ~TIM_CR1_CEN; // Stop timer
        mode = 1;
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
}

void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR13){
        start = !start;
        TIM4->CR1 |= TIM_CR1_CEN;
        EXTI->PR |= EXTI_PR_PR13; // Clear pending bit
    }
}

void PWM_Output_PC6_Init(void){
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    TIM8->CR1 &= ~TIM_CR1_CEN;    
    // 2. Set Timer Frequency (1MHz clock, 20ms period)
    TIM8->PSC= (SYSTEM_FREQ / 1000000) - 1;
    TIM8->ARR= 19999;

    // 3. Configure TIM8 Channel 1 for PWM Mode 1
    TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM8->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;
    
    // 4. Enable Output and Main Output Enable
    TIM8->CCER |= TIM_CCER_CC4E;
    TIM8->BDTR |= TIM_BDTR_MOE;

    TIM8->CNT = 0;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;
}

int main(void){
    //Initialize ultrasonic sensor
    init_ultrasound();
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

    init_gp_timer(TIM4, 1000, 0xFFFF, false);
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
    SERVO_t ultrasound_servo= {
        .SERVO_PIN_PORT = GPIOC,
        .SERVO_PWM_PIN = 6,
        .SERVO_FEEDBACK_PIN = 16 // Does not exist, placeholder
    };
    init_servo(&ultrasound_servo);
    init_servo(&left_wheel);
    init_servo(&right_wheel);

    PWM_Output_PC6_Init();

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
        display_num(TIM4->CNT/100, 1);
        read_uv_sensors();
        if(start && !mode){
            blank_drive();
        }else if(start && mode){
            direction();
        }else{
            TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
            TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
        }
    }
    return 0;
}