#ifndef DRIVE_H
#define DRIVE_H
#include <stdint.h>

#define SERVO_NEUTRAL_PULSE_WIDTH 1500
#define CW_MAX_PULSE 1480 
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720

void blank_drive(volatile uint8_t* mode);
void read_uv_sensors(volatile uint16_t* sensor);
void move_forward(void);
void turn_right(void);
void turn_left(void);

#endif // DRIVE_H