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
            //printf("%d ", (int)(m->matrix[i][j] * 1000));
            printf("%f ", m->matrix[i][j]);
        }
        printf("\n");
    }
}

void embedi_create_matrix(struct matrix *m, unsigned char row, unsigned char colum)
{
#if 1
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
#endif
}

void embedi_reset_matrix(struct matrix *m)
{
    if (!m) {
        printf("failed to create matrix \n");
        return;
    }
    for (int i = 0; i < m->row; i++) {
        for (int j = 0; j < m->colum; j++) {
            m->matrix[i][j] = 0;
        }
    }
    m->row = 0;
    m->colum = 0;
}

void embedi_copy_matrix(struct matrix *from, struct matrix *to)
{
    if (!from || !to ||
        from->colum != to->colum ||
        from->row != to->row) {
        printf("copy para invalid \n");
        return;
    }

    for (int i = 0; i < to->row; i++) {
        for (int j = 0; j < to->colum; j++) {
            to->matrix[i][j] = from->matrix[i][j];
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
    float tmp[4][4];

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
            tmp[i][j] = a->matrix[i][j] + b->matrix[i][j];
        }
    }

    for (int i = 0; i < a->row; i++) {
        for (int j = 0; j < a->colum; j++) {
            r->matrix[i][j] = tmp[i][j];
        }
    }
}

void embedi_matrix_minus(struct matrix *a, struct matrix *b, struct matrix *r)
{
    float tmp[4][4];

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
            tmp[i][j] = a->matrix[i][j] - b->matrix[i][j];
        }
    }

    for (int i = 0; i < a->row; i++) {
        for (int j = 0; j < a->colum; j++) {
            r->matrix[i][j] = tmp[i][j];
        }
    }
}

void embedi_matrix_scale(struct matrix *a, float constant, struct matrix *r)
{
    for (int i = 0; i < a->row; i++) {
        for (int j = 0; j < a->colum; j++) {
            r->matrix[i][j] = a->matrix[i][j] * constant;
        }
    }
}

void embedi_matrix_mul(struct matrix *a, struct matrix *b, struct matrix *r)
{
    float tmp[4][4];

    if (!a || !b || !r) {
        printf("matrix null\n");
        return;
    }

    if ((a->colum != b->row) ||
        (a->row != r->row) || (b->colum != r->colum)) {
        printf("failed to do matrix mul %d %d %d %d %d %d\n",
               a->colum, b->row, a->row, r->row, b->colum, r->colum);
        return;
    }

    for (int i = 0; i < a->row; i++) {
        for (int j = 0; j < b->colum; j++) {
            float sum = 0;
            for (int k = 0; k < a->colum; k++) {
                sum += a->matrix[i][k] * b->matrix[k][j];
            }
            tmp[i][j] = sum;
            // printf("t %d ", (int)(tmp[i][j] * 1000));
        }
    }

    // printf("\n");

    for (int i = 0; i < r->row; i++) {
        for (int j = 0; j < r->colum; j++) {
            r->matrix[i][j] = tmp[i][j];
            // printf("r %d ", (int)(r->matrix[i][j] * 1000));
        }
    }
    // printf("\n");
}

void embedi_matrix_2x2_inv(struct matrix *a, struct matrix *inv)
{
    double det = a->matrix[0][0] * a->matrix[1][1] - a->matrix[0][1] * a->matrix[1][0];
    if (det == 0) {
        printf("det is 0\n");
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
        printf("det is 0\n");
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
#if 1
    if (!a || !T) {
        printf("matrix null\n");
        return;
    }

    for (int i = 0; i < T->row; i++) {
        for (int j = 0; j < T->colum; j++) {
            T->matrix[i][j] = a->matrix[j][i];
            // printf("%d %d\n", i, j);
        }
    }
#endif
}

void embedi_matrix_test(void)
{
    struct matrix a, b, c;
    struct matrix r;
    struct matrix r_inv;
    struct matrix T;

    embedi_create_matrix(&a, 2, 2);
    a.matrix[0][0] = 5;
    a.matrix[0][1] = 8;
    a.matrix[1][0] = 4;
    a.matrix[1][1] = 5;
    // embedi_print_matrix(&a);
    // printf("\n");

    embedi_create_matrix(&b, 2, 1);
    b.matrix[0][0] = 5;
    b.matrix[1][0] = 4;

    embedi_create_matrix(&c, 2, 1);
    c.matrix[0][0] = 7;
    c.matrix[1][0] = 8;
    embedi_matrix_add(&b, &c, &c);
    ;
    // embedi_print_matrix(&c);
    //  printf("\n");

    embedi_matrix_mul(&a, &b, &b);
    // embedi_print_matrix(&b);

    embedi_create_matrix(&r, 3, 3);
    embedi_create_matrix(&r_inv, 3, 3);
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
    //  printf("\n");

    embedi_matrix_scale(&r_inv, 10, &r_inv);
    // embedi_print_matrix(&r_inv);
    //  printf("\n");

    struct matrix vector;
    embedi_create_matrix(&vector, 3, 1);
    vector.matrix[0][0] = 1;
    vector.matrix[1][0] = 2;
    vector.matrix[2][0] = 3;
    embedi_create_matrix(&T, 1, 3);
    embedi_matrix_transpose(&vector, &T);
    // embedi_print_matrix(&vector);
    // embedi_print_matrix(&T);
    //  printf("\n");

    embedi_create_matrix(&a, 2, 2);
    embedi_create_matrix(&r_inv, 2, 2);
    a.matrix[0][0] = 1;
    a.matrix[0][1] = 5;
    a.matrix[1][0] = 1;
    a.matrix[1][1] = 4;
    embedi_matrix_2x2_inv(&a, &r_inv);
    // embedi_print_matrix(&r_inv);
    // printf("\n");
}

/* taylor expansion arctanx = x - x^3/3  + x^5/5 */
float embedi_arctan(float x)
{
    float talor_r = x - (x * x * x / 3) + (x * x * x * x * x) / 5;

    return talor_r;
}

/* taylor expansion arcsin = x + x^3/6  + 3*x^5/100 */
float embedi_arcsin(float x)
{
    float talor_r = x - (x * x * x / 6) + 3*(x * x * x * x * x) / 40;

    return talor_r;
}
