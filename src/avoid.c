#include "avoid.h"
#include "drive.h"
#include "hat.h"
#include <stdint.h>

void avoid_wall(void){
    uint32_t left,center,right;

    TIM8->CCR1 = SERVO_LEFT;
    delay_us(100);
    left=ultrasonic_measure();

    TIM8->CCR1 = SERVO_CENTER;
    delay_us(100);
    center = ultrasonic_measure();

    TIM8->CCR1=SERVO_RIGHT;
    delay_us(100);
    right = ultrasonic_measure();

    if(center > 1000){
        move_forward();
    }else if(left> right){
        turn_left();
    }else{
        turn_right();
    }

    TIM8->CCR1 = SERVO_CENTER;    // Reset sensor to center
}