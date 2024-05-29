/* not compiler yet*/
#include "embedi_config.h"
#include "embedi_math.h"
#include "embedi_scope.h"
#include <math.h>
#include <stdio.h>

#define KALMAN_R 100
#define KALMAN_Q 0.0025
#define KALMAN_DT 0.005

enum emebedi_xyz {
    X = 0,
    Y,
    Z,
};

struct _kalman_para {
    float dt;
    struct matrix A;
    struct matrix B;
    struct matrix H;
    struct matrix Q;
    struct matrix R;
};

/*
    linear system status equation:
    .
    x = A(t) * x + B(t) *u
    y = H(t) * x
*/
struct embedi_kalman {
    /*
        if it's not a linear system:
        you should implement your status equation
        @ x_equation
        @ y_equation
    */
    void (*x_equation)(void);
    void (*y_equation)(void);
    struct matrix observer;
    struct matrix P;
    struct matrix K;
    struct _kalman_para para;
};

static struct embedi_kalman filter;

static void _q_norm(float *q)
{
    float mag;
    mag = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (mag) {
        q[0] /= mag;
        q[1] /= mag;
        q[2] /= mag;
        q[3] /= mag;
    } else {
        q[0] = 1.f;
        q[1] = 0.f;
        q[2] = 0.f;
        q[3] = 0.f;
    }
}

static void _v_norm(float *v)
{
    float mag;
    mag = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (mag) {
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    } else {
        v[0] = 1.f;
        v[1] = 0.f;
        v[2] = 0.f;
    }
}

static void _update_A(float *gyro)
{
    struct matrix E;
    float a = 0;

    embedi_create_eye_matrix(&E, 4, 4);
    a = filter.para.dt / 2;

    filter.para.A.matrix[0][0] = 0;
    filter.para.A.matrix[0][1] = -gyro[X];
    filter.para.A.matrix[0][2] = -gyro[Y];
    filter.para.A.matrix[0][3] = -gyro[Z];

    filter.para.A.matrix[1][0] = gyro[X];
    filter.para.A.matrix[1][1] = 0;
    filter.para.A.matrix[1][2] = gyro[Z];
    filter.para.A.matrix[1][3] = -gyro[Y];

    filter.para.A.matrix[2][0] = gyro[Y];
    filter.para.A.matrix[2][1] = -gyro[Z];
    filter.para.A.matrix[2][2] = 0;
    filter.para.A.matrix[2][3] = gyro[X];

    filter.para.A.matrix[3][0] = gyro[Z];
    filter.para.A.matrix[3][1] = gyro[Y];
    filter.para.A.matrix[3][2] = -gyro[X];
    filter.para.A.matrix[3][3] = 0;

    embedi_matrix_scale(&filter.para.A, a, &filter.para.A);
    embedi_matrix_add(&filter.para.A, &E, &filter.para.A);

    // embedi_print_matrix(&filter.para.A);
}

static void _update_H(void)
{
    filter.para.H.matrix[0][0] = -filter.observer.matrix[2][0];
    filter.para.H.matrix[0][1] = filter.observer.matrix[3][0];
    filter.para.H.matrix[0][2] = -filter.observer.matrix[0][0];
    filter.para.H.matrix[0][3] = filter.observer.matrix[1][0];

    filter.para.H.matrix[1][0] = filter.observer.matrix[1][0];
    filter.para.H.matrix[1][1] = filter.observer.matrix[0][0];
    filter.para.H.matrix[1][2] = filter.observer.matrix[3][0];
    filter.para.H.matrix[1][3] = filter.observer.matrix[2][0];

    filter.para.H.matrix[2][0] = filter.observer.matrix[0][0];
    filter.para.H.matrix[2][1] = -filter.observer.matrix[1][0];
    filter.para.H.matrix[2][2] = -filter.observer.matrix[2][0];
    filter.para.H.matrix[2][3] = filter.observer.matrix[3][0];

    embedi_matrix_scale(&filter.para.H, 2, &filter.para.H);
}

static void _predect(float *gyro)
{
    _update_A(gyro);
    /*
        step1:
        status equation
        observer(k+1) = A*observer(k)
        observer = filter.A * observer;
        observer = [
            q0
            q1
            q2
            q3
        ]
    */
    struct matrix tmp;

    // printf("_predect call \n");
    /*observer = filter.A * observer*/
    embedi_matrix_mul(&filter.para.A, &filter.observer, &filter.observer);

    /*
        step2:
        P equation
        P(k+1) = A*P(k)A(T) + Q
        here P and A Q are 4x4 matrix
    */
    /* tmp = A*P(k)*/
    // embedi_reset_matrix(&tmp);
    embedi_create_matrix(&tmp, 4, 4);
    embedi_matrix_mul(&filter.para.A, &filter.P, &tmp);

    /*A(T)*/
    struct matrix transpose;
    embedi_create_matrix(&transpose, 4, 4);
    embedi_matrix_transpose(&filter.para.A, &transpose);

    /* mul_result = A*P(k)A(T)*/
    struct matrix mul_result;
    embedi_create_matrix(&mul_result, 4, 4);
    embedi_matrix_mul(&tmp, &transpose, &mul_result);

    /* P(k+1) = A*P(k)A(T) + Q */
    embedi_matrix_add(&mul_result, &filter.para.Q, &filter.P);
}

static void _update(float *accel)
{
    _update_H();
    /*
        step3:
        update kalman K
        K = P(k)*H(T) / (H*P(k)*H(T) + R)
    */
    /*H(T): 2x2*/
    // printf("_update call \n");

    struct matrix transpose;
    embedi_create_matrix(&transpose, 4, 3);
    embedi_matrix_transpose(&filter.para.H, &transpose);
    /* tmp1 = P(k) * H(T): 4x3*/
    struct matrix tmp1;
    embedi_create_matrix(&tmp1, 4, 3);
    embedi_matrix_mul(&filter.P, &transpose, &tmp1);

    /* tmp2 = H * P(k) * H(T) : 3x3*/
    struct matrix tmp2;
    embedi_create_matrix(&tmp2, 3, 3);
    embedi_matrix_mul(&filter.para.H, &tmp1, &tmp2);

    /* tmp2 = H * P(k) * H(T) +  R: 3x3*/
    embedi_matrix_add(&tmp2, &filter.para.R, &tmp2);

    /* tmp3 = inv(tmp2) : 3x3*/
    struct matrix tmp3;
    embedi_create_matrix(&tmp3, 3, 3);
    embedi_matrix_3x3_inv(&tmp2, &tmp3);

    /* K = tmp1 * inv(tmp2) : 4x3*/
    embedi_matrix_mul(&tmp1, &tmp3, &filter.K);

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
             acc_x
             acc_y
             acc_z
         ]
    */
    embedi_create_matrix(&z, 3, 1);
    _v_norm(accel);
    z.matrix[0][0] = accel[X];
    z.matrix[1][0] = accel[Y];
    z.matrix[2][0] = accel[Z];

    /* tmp4 = H*observer(k) : 3x1
       in nolinear system use continuous function instead
    */
    struct matrix tmp4;
    float q0, q1, q2, q3;
    q0 = filter.observer.matrix[0][0];
    q1 = filter.observer.matrix[1][0];
    q2 = filter.observer.matrix[2][0];
    q3 = filter.observer.matrix[3][0];

    embedi_create_matrix(&tmp4, 3, 1);
    tmp4.matrix[0][0] = 2 * (q1 * q3 - q0 * q2);
    tmp4.matrix[1][0] = 2 * (q0 * q1 - q2 * q3);
    tmp4.matrix[2][0] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    // embedi_matrix_mul(&filter.para.H, &filter.observer, &tmp4);

    /* tmp5 = z-H*observer(k) : 3x1*/
    struct matrix tmp5;
    embedi_create_matrix(&tmp5, 3, 1);
    embedi_matrix_minus(&z, &tmp4, &tmp5);

    /* tmp6 = K*(z-H*observer(k)) : 4x1*/
    struct matrix tmp6;
    embedi_create_matrix(&tmp6, 4, 1);
    embedi_matrix_mul(&filter.K, &tmp5, &tmp6);

    // get final fusion filer result
    embedi_matrix_add(&filter.observer, &tmp6, &filter.observer);
    // printf("_update step4\n");
    /*
        step5:
        updata p matrix
        P(k+1) = (I - K*H)*P(k)
    */
    struct matrix tmp7;
    struct matrix E;
    /* tmp7 = K*H : 4x4*/
    embedi_create_matrix(&tmp7, 4, 4);
    embedi_matrix_mul(&filter.K, &filter.para.H, &tmp7);

    /* tmp7 = E-K*H : 4x4*/
    embedi_create_eye_matrix(&E, 4, 4);
    embedi_matrix_mul(&filter.K, &filter.para.H, &tmp7);
    embedi_matrix_minus(&E, &tmp7, &tmp7);

    /* P(k+1) = (E - K*H)*P(k)*/
    embedi_matrix_mul(&tmp7, &filter.P, &filter.P);
    // printf("_update step5\n");

    float quat[4];
    quat[0] = filter.observer.matrix[0][0];
    quat[1] = filter.observer.matrix[1][0];
    quat[2] = filter.observer.matrix[2][0];
    quat[3] = filter.observer.matrix[3][0];
    _q_norm(quat);
    filter.observer.matrix[0][0] = quat[0];
    filter.observer.matrix[1][0] = quat[1];
    filter.observer.matrix[2][0] = quat[2];
    filter.observer.matrix[3][0] = quat[3];
    // embedi_print_matrix(&filter.observer);
}

void embedi_6d_kalman_filter(float *accel, float *gyro)
{
    _predect(gyro);
    _update(accel);
}

void embedi_6d_kalman_init(void)
{
    filter.para.dt = KALMAN_DT;

    embedi_create_matrix(&filter.para.A, 4, 4);
    embedi_create_matrix(&filter.para.H, 3, 4);
    embedi_create_eye_matrix(&filter.para.Q, 4, 4);
    embedi_matrix_scale(&filter.para.Q, KALMAN_Q, &filter.para.Q);

    embedi_create_eye_matrix(&filter.para.R, 3, 3);
    embedi_matrix_scale(&filter.para.R, KALMAN_R, &filter.para.R);

    embedi_create_matrix(&filter.observer, 4, 1);
    filter.observer.matrix[0][0] = 1;
    filter.observer.matrix[1][0] = 0;
    filter.observer.matrix[2][0] = 0;
    filter.observer.matrix[3][0] = 0;

    embedi_create_eye_matrix(&filter.P, 4, 4);
    embedi_create_eye_matrix(&filter.K, 4, 3);
}

struct matrix *emebedi_get_6d_kalman_obsever(void)
{
    return &filter.observer;
}
/*
    rotation matrix[9]:
    [0][1][2]
    [3][4][5]
    [6][7][8]
*/

void embedi_quat_to_rot(struct matrix *rot)
{
    float a = filter.observer.matrix[0][0];
    float b = filter.observer.matrix[1][0];
    float c = filter.observer.matrix[2][0];
    float d = filter.observer.matrix[3][0];

    if (!rot) {
        printf("rot null");
        return;
    }

    rot->matrix[0][0] = 1 - 2 * c * c - 2 * d * d;
    rot->matrix[0][1] = 2 * (b * c - a * d);
    rot->matrix[0][2] = 2 * (a * c + b * d);
    rot->matrix[1][0] = 2 * (b * c + a * d);
    rot->matrix[1][1] = 1 - 2 * b * b - 2 * d * d;
    rot->matrix[1][2] = 2 * (c * d - a * b);
    rot->matrix[2][0] = 2 * (b * d - a * c);
    rot->matrix[2][1] = 2 * (a * b + c * d);
    rot->matrix[2][2] = 1 - 2 * b * b - 2 * c * c;
}
