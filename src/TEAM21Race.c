#include "hat.h"
#include "drive.h"
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
#define STOP_SPEED 1500

volatile uint8_t mode = 0;
volatile uint16_t sensor = 0;
volatile bool start = false;

volatile uint32_t rise_time = 0;
volatile uint32_t fall_time = 0;
volatile uint32_t pulse_duration = 0;
volatile uint32_t pulse_width = 0; 
volatile float distance = 0.0; // in cm

void init_exti0(void){
    init_gp_timer(TIM2, 1000000, 0xFFFFFFFF, true); // 1MHz timer for microsecond precision
    // Configure EXTI for ultrasound echo pin (PB0)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable SYSCFG clock
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB; // EXTI0 from PB0
    EXTI->IMR |= EXTI_IMR_IM0; // unmask EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0; // rising edge trigger
    EXTI->FTSR |= EXTI_FTSR_TR0; // falling edge trigger
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 0);
}

void SysTick_Handler(void){
    write_pin(ULTRA_SOUND.TRIG_PORT, ULTRA_SOUND.TRIG_PIN, HIGH);
    delay_us(10);
    write_pin(ULTRA_SOUND.TRIG_PORT, ULTRA_SOUND.TRIG_PIN, LOW);
}
void EXTI0_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR0){
        if(ULTRA_SOUND.ECHO_PORT->IDR & (1 << ULTRA_SOUND.ECHO_PIN)){
            rise_time = TIM2->CNT;
        }else{
            fall_time = TIM2->CNT;
            pulse_duration = (fall_time >= rise_time) ? (fall_time - rise_time) : (0xFFFFFFFF - rise_time + fall_time + 1);
            pulse_duration /= 16; // convert to microseconds
            distance = ((pulse_duration) / 58.0); // in cm
        }
        EXTI->PR |= EXTI_PR_PR0;
    }
    return;
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
    
    // 4. Enable Channel 1 Output and Main Output Enable
    TIM8->CCER |= TIM_CCER_CC1E;
    TIM8->BDTR |= TIM_BDTR_MOE;

    TIM8->CCR1 = SERVO_CENTER; // Set initial position
    TIM8->CNT = 0;
    TIM8->EGR = TIM_EGR_UG;

    TIM8->CR1 |= TIM_CR1_CEN;
}

int main(void){
    init_usart(115200);
    //Initialize ultrasonic sensor
    init_sys_tick(8000000);
    init_exti0();
    init_ultrasound();
    // Initialize SSD
    init_ssd(10);
    // Setup TIM3
    init_gp_timer(TIM3, TIM3_FREQ_HZ, PWM_PERIOD, false);
    // PWM mode 1 for CH3 and CH4
    TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) | (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable CH3 and CH4 outputs
    TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH; // PC8
    TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
    TIM3->CR1 |= TIM_CR1_CEN;

    // Initialize and start TIM2 for ultrasonic measurement (1MHz clock)
    init_gp_timer(TIM2, 1000000, 0xFFFFFFFF, false);
    TIM2->CR1 |= TIM_CR1_CEN;

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
    init_servo(&right_wheel);
    init_servo(&left_wheel);

    PWM_Output_PC6_Init();
    init_servo(&ultrasound_servo); // Configures PC6 to AF3 for TIM8

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
        read_uv_sensors(&sensor);
        delay_us(60000); // 60ms between measurements
        char buffer[50];
        sprintf(buffer, "Distance: %.2f cm\r\n", distance);
        send_string(buffer);
        if(start){
            if(distance < 35){
                blank_drive(&mode);
            }else{
                avoid_obstacle(sensor);
            }
        }else{
            TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
            TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
        }
    }
    return 0;
}