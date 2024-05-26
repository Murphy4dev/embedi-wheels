#ifndef __EMBEDI_TEST_H
#define __EMBEDI_TEST_H

enum {
    /* 0 */
    MOTOR_STOP = 48, // scall
    /* 1 */
    MOTOR_START_FORDWARD = 49,
    /* 2 */
    MOTOR_START_BACKWARD = 50,
    /* 3 */
    IMU_CALIBRATION = 51,
    /* 4 */
    IMU_FLASH_WRITE = 52,
    /* 5 */
    IMU_FLASH_READ = 53,
    /* 6 */
    FLASH_WRITE = 54,
    /* 7 */
    FLASH_READ = 55,
    /* 9 */
    RUNNING_SWICH = 57
};

#endif
