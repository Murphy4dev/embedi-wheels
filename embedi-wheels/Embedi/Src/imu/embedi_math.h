#ifndef EMBEDI_MATH_H
#define EMBEDI_MATH_H


#ifdef __cplusplus
extern "C" {
#endif

#define MATRIX_SIZE 4

struct matrix {
    unsigned char row;
    unsigned char colum;
    float matrix[MATRIX_SIZE][MATRIX_SIZE];
};
void embedi_print_matrix(struct matrix *m);
void embedi_create_matrix(struct matrix *m,  unsigned char row,  unsigned char colum);
void embedi_create_eye_matrix(struct matrix *m, unsigned char row, unsigned char colum);
void embedi_reset_matrix(struct matrix *m);
void embedi_matrix_add(struct matrix *a, struct matrix *b, struct matrix *r);
void embedi_matrix_minus(struct matrix *a, struct matrix *b, struct matrix *r);
void embedi_matrix_mul(struct matrix *a, struct matrix *b, struct matrix *r);
void embedi_matrix_scale(struct matrix *a, float constant, struct matrix *r);
void embedi_matrix_transpose(struct matrix *a, struct matrix *T);
void embedi_matrix_2x2_inv(struct matrix *a, struct matrix *inv);
void embedi_matrix_3x3_inv(struct matrix *a, struct matrix *inv);
void embedi_matrix_test(void);

#ifdef __cplusplus
}
#endif
#endif // EMBEDI_MATH_H
