#include "avoid.h"
#include "drive.h"
#include "hat.h"
#include "usMeasure.h"
#include <stdint.h>

void avoid_wall(void){
    uint32_t left,center,right;

    TIM8->CCR1 = SERVO_LEFT;
    delay_us(100);
    trigger_pulse();
    left = distance;

    TIM8->CCR1 = SERVO_CENTER;
    delay_us(100);
    trigger_pulse();
    right = distance;

    TIM8->CCR1=SERVO_RIGHT;
    delay_us(100);
    trigger_pulse();
    center = distance;

    if(center > 1000){
        move_forward();
    }else if(left> right){
        turn_left();
    }else{
        turn_right();
    }

    TIM8->CCR1 = SERVO_CENTER;    // Reset sensor to center
}