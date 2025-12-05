#include "usmeasure.h"
#include "hat.h"

static volatile uint32_t rise_time = 0;
static volatile uint32_t fall_time = 0;
static volatile uint32_t pulse_duration = 0;
static volatile uint32_t pulse_width = 0; 


void init_exti6(void){
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable SYSCFG clock
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB; // EXTI6 from PB6
    EXTI->IMR |= EXTI_IMR_IM6; // unmask EXTI6
    EXTI->RTSR |= EXTI_RTSR_TR6; // rising edge trigger
    EXTI->FTSR |= EXTI_FTSR_TR6; // falling edge trigger
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 0);
}

void trigger_pulse(void){ //sends a 10us pulse to the trigger pin
    write_pin(ULTRA_SOUND.TRIG_PORT, ULTRA_SOUND.TRIG_PIN, HIGH);
    delay_us(10);
    write_pin(ULTRA_SOUND.TRIG_PORT, ULTRA_SOUND.TRIG_PIN, LOW);
}

void EXTI5_9_IRQHandler(void){ 
    if(EXTI->PR & EXTI_PR_PR0){
        if(ULTRA_SOUND.ECHO_PORT->IDR & (1 << ULTRA_SOUND.ECHO_PIN)){
            rise_time = TIM2->CNT;
        }else{
            fall_time = TIM2->CNT;
            pulse_duration = (fall_time >= rise_time) ? (fall_time - rise_time) : (0xFFFFFFFF - rise_time + fall_time + 1);
            pulse_duration /= 16; // convert to microseconds
            pulse_width= pulse_duration; //to store echo pulse width
            distance = ((pulse_duration) / 58.0); // in cm
        }
        EXTI->PR |= EXTI_PR_PR0;
    }
    return;
}