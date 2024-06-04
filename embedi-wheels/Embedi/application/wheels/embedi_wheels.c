#include "embedi_wheels.h"
#include "cmsis_os.h"
#include "embedi_2d_kalman.h"
#include "embedi_6d_kalman.h"
#include "embedi_delay.h"
#include "embedi_flash.h"
#include "embedi_i2c.h"
#include "embedi_imu.h"
#include "embedi_module_init.h"
#include "embedi_motor.h"
#include "embedi_pid.h"
#include "embedi_scope.h"
#include "embedi_system.h"
#include "embedi_test.h"
#include "main.h"
#include <stdio.h>
#ifdef CFG_UART_ENABLE
#include "embedi_uart.h"
#endif

// #define TEST_MODE
#define USE_6D_ALGO
#define rad2deg (57.2957)
/* banlance*/
#define BALANCE_P (600 * rad2deg)
#define BALANCE_D (10 * rad2deg)
#define BALANCE_T (1.75 / rad2deg)
/* banlance*/
#define VELOCITY_P (80) // 25.0
#define VELOCITY_I (4)  // 3.25
#define VELOCITY_D (0)
#define VELOCITY_LIMIT (10000) // 3.25
#define VELOCITY_T (0)
osThreadId _task_handle;
void embedi_task(void const *argument);
SemaphoreHandle_t xSemaphore = NULL;

static struct _pid balance_pd;
static struct _pid velocity_pi;
static float balance_pwm = 0;
static float velocity_pwm = 0;
static float velocity = 0;
static float velocity_last = 0;
static float abnormal_protect = 0;

void imu_irq_cb(int irq_num)
{
    if (irq_num == IMU_IRQ_GPIO) {
        if (xSemaphore) {
            xSemaphoreGiveFromISR(xSemaphore, NULL);
        }
    }
}
irq_register(imu_irq_cb);

void key_irq_cb(int irq_num)
{
    if (irq_num == KEY_IRQ_GPIO) {
        printf("START key press \n");
        embedi_set_run_state(_START);
    }
}
irq_register(key_irq_cb);

static void _reset_parameters(void)
{
    balance_pwm = 0;
    velocity_pwm = 0;
    velocity = 0;
    velocity_last = 0;
    abnormal_protect = 0;
    embedi_pid_reset(&balance_pd);
    embedi_pid_reset(&velocity_pi);
}

static void abnormal_check(void)
{
    if (velocity > 50 || velocity < -50) {
        embedi_motor_sotp();
        abnormal_protect = 1;
    }
}

static void _wheels_control(void)
{
    int r_speed = 0;
    int l_speed = 0;
#ifndef USE_6D_ALGO
    float angle;
#else
    float euler_angle[3];
#endif
    float pwm = 0;
#ifndef USE_6D_ALGO
    embedi_get_roll_angle(&angle);
    balance_pwm = -embedi_pid(&balance_pd, angle);
#else
    embedi_get_euler_angle(euler_angle);
    balance_pwm = -embedi_pid(&balance_pd, euler_angle[0], 0);
#endif
    embedi_get_speed(&r_speed, &l_speed);
    // lowpass filter
    velocity_last = velocity;
    velocity = r_speed + l_speed;
    velocity = velocity_last * 0.86 + velocity * 0.14;

    velocity_pwm = -embedi_pid(&velocity_pi, velocity, VELOCITY_LIMIT);
    pwm = balance_pwm + velocity_pwm;
#ifdef CFG_IMU_DATA_SCOPE_SHOW
#if (VELOCITY_DATA_SHOW == 1)
    // embedi_data_to_scope(pwm, CHANNEL_1);
    // embedi_data_to_scope(balance_pwm, CHANNEL_2);
    // embedi_data_to_scope(velocity_pwm, CHANNEL_3);
    // embedi_data_to_scope((r_speed + l_speed) / 2, CHANNEL_3);
    // embedi_scope_show();
    printf("%d %d %d v:%d\n", (int)pwm, (int)balance_pwm, (int)velocity_pwm, (int)velocity);
#endif
#endif
    abnormal_check();
    if (!abnormal_protect) {
        if (pwm > 0) {
            embedi_set_direction(FORDWARD);
        } else {
            embedi_set_direction(BACKWARD);
        }
        embedi_motor_start((int)pwm, (int)pwm);
    }
}

static void _idle_heart_beat(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    osDelay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    osDelay(200);
}

void embedi_task_function(void const *argument)
{
    // need to IMU systick
    embedi_imu_enable();
    for (;;) {
        if (xSemaphoreTake(xSemaphore, 0) == pdTRUE) {
            switch (embedi_get_run_state()) {
            case _START:
                _wheels_control();
                break;
            case IMU_CALIBRATION: {
                int data_ready = 0;
                data_ready = embedi_update_imu_data_buff();
                if (data_ready) {
                    embedi_imu_calibration();
                    embedi_set_run_state(_DEFAULT);
                }
                break;
            }
            default:
                _idle_heart_beat();
                embedi_motor_sotp();
                _reset_parameters();
                break;
            }
        }
    }
}

osTimerId wheels_handle;
void timer_function(void const *argument)
{
    if (xSemaphore) {
        xSemaphoreGiveFromISR(xSemaphore, NULL);
    }
}

void embedi_wheels_init(void)
{
    balance_pwm = 0;
    velocity_pwm = 0;

    embedi_pid_init(&balance_pd, BALANCE_T, BALANCE_P, 0, BALANCE_D);
    embedi_pid_init(&velocity_pi, VELOCITY_T, VELOCITY_P, VELOCITY_I, VELOCITY_D);

    xSemaphore = xSemaphoreCreateBinary();
    osThreadDef(embedi_task, embedi_task_function, osPriorityBelowNormal, 0, 512);
    _task_handle = osThreadCreate(osThread(embedi_task), NULL);

    if (!IMU_IRQ_GPIO) {
        osTimerDef(wheels_timer, timer_function);
        wheels_handle = osTimerCreate(osTimer(wheels_timer), osTimerPeriodic, NULL);
        osTimerStart(wheels_handle, 5);
        printf("start 5ms timer \n");
    }
}

app_init(embedi_wheels_init);
