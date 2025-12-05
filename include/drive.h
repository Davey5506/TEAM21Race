#ifndef DRIVE_H
#define DRIVE_H
#include <stdint.h>

#define SERVO_NEUTRAL_PULSE_WIDTH 1500
#define CW_MAX_PULSE 1480 
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720
#define SERVO_RIGHT 1000
#define SERVO_CENTER 1500
#define SERVO_LEFT 2000

void blank_drive(volatile uint8_t* mode);
void avoid_obstacle(volatile uint16_t sensor);
void read_uv_sensors(volatile uint16_t* sensor);
void move_forward(void);
void turn_right(void);
void turn_left(void);

#endif // DRIVE_H