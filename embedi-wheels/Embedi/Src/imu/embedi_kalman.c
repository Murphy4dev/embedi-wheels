#include "embedi_config.h"
#include "embedi_math.h"
#include "embedi_scope.h"
#include <stdio.h>

#define KALMAN_R 100
#define KALMAN_Q 0.001
#define KALMAN_DT 0.005

struct _kalman {
    float dt;
    struct matrix A;
    struct matrix B;
    struct matrix H;
    struct matrix Q;
    struct matrix R;
};

static struct _kalman filter;
static struct matrix observer;
static struct matrix P;
static struct matrix K;

static void _predect(float gyro)
{
    /*
        step1:
        status equation
        observer(k+1) = A*observer(k) + B*U(k)
        observer = filter.A * observer + filter.B * gyro;
        observer = [
            angle
            gyro_bias
        ]
    */
    struct matrix tmp;

    // printf("_predect call \n");
    /*observer = filter.A * observer*/
    embedi_matrix_mul(&filter.A, &observer, &observer);

    /*tmp = filter.B * gyro*/
    embedi_create_matrix(&tmp, 2, 1);
    embedi_matrix_scale(&filter.B, gyro, &tmp);

    /*observer = observer + tmp*/
    embedi_matrix_add(&observer, &tmp, &observer);

    /*
        step2:
        P equation
        P(k+1) = A*P(k)A(T) + Q
        here P and A Q are 2x2 matrix
    */
    /* tmp = A*P(k)*/
    // embedi_reset_matrix(&tmp);
    embedi_create_matrix(&tmp, 2, 2);
    embedi_matrix_mul(&filter.A, &P, &tmp);

    /*A(T)*/
    struct matrix transpose;
    embedi_create_matrix(&transpose, 2, 2);
    embedi_matrix_transpose(&filter.A, &transpose);

    /* mul_result = A*P(k)A(T)*/
    struct matrix mul_result;
    embedi_create_matrix(&mul_result, 2, 2);
    embedi_matrix_mul(&tmp, &transpose, &mul_result);

    /* P(k+1) = A*P(k)A(T) + Q */
    embedi_matrix_add(&mul_result, &filter.Q, &P);
}

static void _update(float accel_angle)
{
    /*
        step3:
        update kalman K
        K = P(k)*H(T) / (H*P(k)*H(T) + R)
    */
    /*H(T): 2x2*/
    // printf("_update call \n");

    struct matrix transpose;
    embedi_create_matrix(&transpose, 2, 2);
    embedi_matrix_transpose(&filter.H, &transpose);
    /* tmp1 = P(k) * H(T): 2x2*/
    struct matrix tmp1;
    embedi_create_matrix(&tmp1, 2, 2);
    embedi_matrix_mul(&P, &transpose, &tmp1);

    /* tmp2 = H * P(k) * H(T) : 2x2*/
    struct matrix tmp2;
    embedi_create_matrix(&tmp2, 2, 2);
    embedi_matrix_mul(&filter.H, &tmp1, &tmp2);

    /* tmp2 = H * P(k) * H(T) + R: 2x2*/
    embedi_matrix_add(&tmp2, &filter.R, &tmp2);

    /* tmp3 = inv(tmp2) : 2x2*/
    struct matrix tmp3;
    embedi_create_matrix(&tmp3, 2, 2);
    embedi_matrix_2x2_inv(&tmp2, &tmp3);

    /* K = tmp1 * inv(tmp2) */
    embedi_matrix_mul(&tmp1, &tmp3, &K);

    // printf("_update step3\n");
    /*
        step4:
        caculate posteriori estimation
        observer(k+1) = observer(k) + K(z-H*observer(k))
    */
    struct matrix z;
    /*
        z is measure state
        measurement = [
             angle
             0
         ]
    */
    embedi_create_matrix(&z, 2, 1);
    z.matrix[0][0] = accel_angle;

    /* tmp4 = H*observer(k) : 2x1*/
    struct matrix tmp4;
    embedi_create_matrix(&tmp4, 2, 1);
    embedi_matrix_mul(&filter.H, &observer, &tmp4);

    /* tmp5 = z-H*observer(k) : 2x1*/
    struct matrix tmp5;
    embedi_create_matrix(&tmp5, 2, 1);
    embedi_matrix_minus(&z, &tmp4, &tmp5);

    /* tmp6 = K*(z-H*observer(k)) : 2x1*/
    struct matrix tmp6;
    embedi_create_matrix(&tmp6, 2, 1);
    embedi_matrix_mul(&K, &tmp5, &tmp6);

    // get final fusion filer result
    embedi_matrix_add(&observer, &tmp6, &observer);
    // printf("_update step4\n");
    /*
        step5:
        updata p matrix
        P(k+1) = (I - K*H)*P(k)
    */
    struct matrix tmp7;
    struct matrix E;
    /* tmp7 = K*H : 2x2*/
    embedi_create_matrix(&tmp7, 2, 2);
    embedi_matrix_mul(&K, &filter.H, &tmp7);

    /* tmp7 = E-K*H : 2x2*/
    embedi_create_eye_matrix(&E, 2, 2);
    embedi_matrix_mul(&K, &filter.H, &tmp7);
    embedi_matrix_minus(&E, &tmp7, &tmp7);

    /* P(k+1) = (I - K*H)*P(k)*/
    embedi_matrix_mul(&tmp7, &P, &P);
    // printf("_update step5\n");
}

/* taylor expansion arctanx = x - x^3/3  + x^5/5 */
static float _arctan(float x)
{
    float talor_r = x - (x * x * x / 3) + (x * x * x * x * x) / 5;

    return talor_r;
}

void embedi_kalman_filter(float accel_y, float accel_z, float gyro)
{
    _predect(gyro);

    float accel_angle = _arctan(accel_y / accel_z);
    _update(accel_angle);
#ifdef CFG_IMU_DATA_SCOPE_SHOW
#if (ROLL_ANGLE_SHOW == 1)
    embedi_data_to_scope(accel_angle * 1000, CHANNEL_1);
    embedi_data_to_scope(observer.matrix[0][0] * 1000, CHANNEL_2);
    embedi_data_to_scope(observer.matrix[1][0] * 1000, CHANNEL_3);
    embedi_scope_show();
#endif
#else
    printf("%d %d %d\n", (int)(accel_angle * 1000),
           (int)(observer.matrix[0][0] * 1000),
           (int)(observer.matrix[1][0] * 1000));
#endif
}

void embedi_kalman_init(void)
{
    filter.dt = KALMAN_DT;

    embedi_create_eye_matrix(&filter.A, 2, 2);
    filter.A.matrix[0][1] = -KALMAN_DT;
    // embedi_print_matrix(&filter.A);

    embedi_create_matrix(&filter.B, 2, 1);
    filter.B.matrix[0][0] = KALMAN_DT;

    embedi_create_matrix(&filter.H, 2, 2);
    filter.H.matrix[0][0] = 1;

    embedi_create_eye_matrix(&filter.Q, 2, 2);
    embedi_matrix_scale(&filter.Q, KALMAN_Q, &filter.Q);

    embedi_create_eye_matrix(&filter.R, 2, 2);
    embedi_matrix_scale(&filter.R, KALMAN_R, &filter.R);

    embedi_create_matrix(&observer, 2, 1);
    observer.matrix[0][0] = 0.1;
    observer.matrix[1][0] = 0.001;
    // embedi_print_matrix(&observer);
    embedi_create_eye_matrix(&P, 2, 2);
    // embedi_print_matrix(&P);
    embedi_create_eye_matrix(&K, 2, 2);
    // embedi_print_matrix(&K);

    // embedi_matrix_test();
}
