#include "embedi_math.h"
#include "embedi_config.h"
#include <stdio.h>

void embedi_print_matrix(struct matrix *m)
{
    if (!m) {
        printf("matrix null \n");
        return;
    }

    for (int i = 0; i < m->row; i++) {
        for (int j = 0; j < m->colum; j++) {
            printf("%f ", m->matrix[i][j]);
        }
        printf("\n");
    }
}

void embedi_create_matrix(struct matrix *m, unsigned char row, unsigned char colum)
{
    if (!m || row > 4 || colum > 4) {
        printf("failed to create matrix \n");
        return;
    }
    m->row = row;
    m->colum = colum;
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < colum; j++) {
            m->matrix[i][j] = 0;
        }
    }
}

void embedi_create_eye_matrix(struct matrix *m, unsigned char row, unsigned char colum)
{
    if (!m || row > 4 || colum > 4) {
        printf("failed to create matrix \n");
        return;
    }
    m->row = row;
    m->colum = colum;
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < colum; j++) {
            if (i == j) {
                m->matrix[i][j] = 1;
            } else {
                m->matrix[i][j] = 0;
            }
        }
    }
}

void embedi_matrix_add(struct matrix *a, struct matrix *b, struct matrix *r)
{
    if (!a || !b || !r) {
        printf("matrix null\n");
        return;
    }

    if ((a->colum != b->colum) || (a->row != b->row) ||
        (a->colum != r->colum) || (a->row != r->row)) {
        printf("failed to do matrix add \n");
        return;
    }

    for (int i = 0; i < a->row; i++) {
        for (int j = 0; j < a->colum; j++) {
            r->matrix[i][j] = a->matrix[i][j] + b->matrix[i][j];
        }
    }
}

void embedi_matrix_scale(struct matrix *a, float constant)
{
    for (int i = 0; i < a->row; i++) {
        for (int j = 0; j < a->colum; j++) {
            a->matrix[i][j] *= constant;
        }
    }
}

void embedi_matrix_mul(struct matrix *a, struct matrix *b, struct matrix *r)
{
    if (!a || !b || !r) {
        printf("matrix null\n");
        return;
    }

    if ((a->colum != b->row) ||
        (a->row != r->row) || (b->colum != r->colum)) {
        printf("failed to do matrix add \n");
        return;
    }

    for (int i = 0; i < a->row; i++) {
        for (int j = 0; j < b->colum; j++) {
            int sum = 0;
            for (int k = 0; k < a->colum; k++) {
                sum += a->matrix[i][k] * b->matrix[k][j];
            }
            r->matrix[i][j] = sum;
        }
    }
}

void embedi_matrix_2x2_inv(struct matrix *a, struct matrix *inv)
{
    double det = a->matrix[0][0] * a->matrix[1][1] - a->matrix[0][1] * a->matrix[1][0];
    if (det == 0) {
        printf("ailed to do matrix inv \n");
        return;
    }

    inv->matrix[0][0] = a->matrix[1][1] / det;
    inv->matrix[0][1] = -a->matrix[0][1] / det;
    inv->matrix[1][0] = -a->matrix[1][0] / det;
    inv->matrix[1][1] = a->matrix[0][0] / det;
}

void embedi_matrix_3x3_inv(struct matrix *a, struct matrix *inv)
{
    int i, j;

    if (!a || !inv) {
        printf("matrix null\n");
        return;
    }

    if (a->colum != a->row || a->row != 3) {
        printf("failed to do matrix inv \n");
        return;
    }

    float det = a->matrix[0][0] * (a->matrix[1][1] * a->matrix[2][2] - a->matrix[2][1] * a->matrix[1][2]) -
                a->matrix[1][0] * (a->matrix[0][1] * a->matrix[2][2] - a->matrix[2][1] * a->matrix[0][2]) +
                a->matrix[2][0] * (a->matrix[0][1] * a->matrix[1][2] - a->matrix[1][1] * a->matrix[0][2]);

    if (!det) {
        printf("matrix inv not exist\n");
        return;
    }
    // Mij
    inv->matrix[0][0] = (a->matrix[1][1] * a->matrix[2][2] - a->matrix[2][1] * a->matrix[1][2]);
    inv->matrix[0][1] = -(a->matrix[0][1] * a->matrix[2][2] - a->matrix[2][1] * a->matrix[0][2]);
    inv->matrix[0][2] = (a->matrix[0][1] * a->matrix[1][2] - a->matrix[1][1] * a->matrix[0][2]);
    inv->matrix[1][0] = -(a->matrix[1][0] * a->matrix[2][2] - a->matrix[2][0] * a->matrix[1][2]);
    inv->matrix[1][1] = (a->matrix[0][0] * a->matrix[2][2] - a->matrix[2][0] * a->matrix[0][2]);
    inv->matrix[1][2] = -(a->matrix[0][0] * a->matrix[1][2] - a->matrix[1][0] * a->matrix[0][2]);
    inv->matrix[2][0] = (a->matrix[1][0] * a->matrix[2][1] - a->matrix[2][0] * a->matrix[1][1]);
    inv->matrix[2][1] = -(a->matrix[0][0] * a->matrix[2][1] - a->matrix[2][0] * a->matrix[0][1]);
    inv->matrix[2][2] = (a->matrix[0][0] * a->matrix[1][1] - a->matrix[1][0] * a->matrix[0][1]);

    // inv
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            inv->matrix[i][j] /= det;
        }
    }
}

void embedi_matrix_transpose(struct matrix *a, struct matrix *T)
{
    if (!a || !T) {
        printf("matrix null\n");
        return;
    }

    for (int i = 0; i < a->row; i++) {
        for (int j = 0; j < a->colum; j++) {
            T->matrix[i][j] = a->matrix[j][i];
        }
    }
}

void embedi_matrix_test(void)
{
    struct matrix a, b;
    struct matrix r;
    struct matrix r_inv;
    struct matrix T;

    embedi_create_matrix(&a, 3, 2);
    a.matrix[0][0] = 5;
    a.matrix[0][1] = 8;
    a.matrix[1][0] = 4;
    a.matrix[1][1] = 5;
    a.matrix[2][0] = 7;
    a.matrix[2][1] = 8;
    // embedi_print_matrix(&a);
    // printf("\n");

    embedi_create_matrix(&b, 2, 3);
    b.matrix[0][0] = 5;
    b.matrix[0][1] = 4;
    b.matrix[0][2] = 3;
    b.matrix[1][0] = 2;
    b.matrix[1][1] = 8;
    b.matrix[1][2] = 10;
    // embedi_print_matrix(&b);
    // printf("\n");

    embedi_create_matrix(&r, 3, 3);
    embedi_create_matrix(&r_inv, 3, 3);

    embedi_matrix_mul(&a, &b, &r);
    // embedi_print_matrix(&r);

    r.matrix[0][0] = 1;
    r.matrix[0][1] = 2;
    r.matrix[0][2] = 3;
    r.matrix[1][0] = 2;
    r.matrix[1][1] = 2;
    r.matrix[1][2] = 1;
    r.matrix[2][0] = 3;
    r.matrix[2][1] = 4;
    r.matrix[2][2] = 3;
    // embedi_print_matrix(&r);
    // printf("\n");

    embedi_matrix_3x3_inv(&r, &r_inv);
    // embedi_print_matrix(&r_inv);
    // printf("\n");

    embedi_matrix_scale(&r_inv, 10);
    // embedi_print_matrix(&r_inv);
    // printf("\n");

    embedi_create_matrix(&T, 3, 3);
    embedi_matrix_transpose(&r_inv, &T);
    // embedi_print_matrix(&T);
    // printf("\n");

    embedi_create_matrix(&a, 2, 2);
    embedi_create_matrix(&r_inv, 2, 2);
    a.matrix[0][0] = 1;
    a.matrix[0][1] = 5;
    a.matrix[1][0] = 1;
    a.matrix[1][1] = 4;
    embedi_matrix_2x2_inv(&a, &r_inv);
    embedi_print_matrix(&r_inv);
    printf("\n");
}
