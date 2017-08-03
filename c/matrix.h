#ifndef __MATRIX_H__
#define __MATRIX_H__

void mat_print(int rows, int cols, const double *data, const char *desc);
void mat_mult(int lrows, int lcols, int rcols, const double *left,
                                               const double *right,
                                               double *result);

#endif
