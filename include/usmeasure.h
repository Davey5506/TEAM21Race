#ifndef USMEASURE_H
#define USMEASURE_H

volatile float distance = 0.0;

void init_exti6(void);
void trigger_pulse(void);

#endif // USMEASURE_H