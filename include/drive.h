#ifndef DRIVE_H
#define DRIVE_H
#include <stdint.h>

#define SERVO_NEUTRAL_PULSE_WIDTH 1500
#define CW_MAX_PULSE 1480 
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720

void blank_drive(uint8_t* mode);
void read_uv_sensors(uint16_t* sensor);

#endif // DRIVE_H