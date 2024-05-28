#ifndef __EMBEDI_MOTOR_H
#define __EMBEDI_MOTOR_H

enum embedi_direction {
    BACKWARD, 
    FORDWARD
};

void motor_test(void);
void embedi_set_direction(enum embedi_direction dir);
void embedi_get_speed(int *right, int *left);
void embedi_motor_start(int left, int right);
void embedi_motor_sotp(void);
#endif
