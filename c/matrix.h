#ifndef __MATRIX_H__
#define __MATRIX_H__

const double *mat_add(int rows, int cols, const double *left, const double *right, double *result);
const double *mat_sub(int rows, int cols, const double *left, const double *right, double *result);
const double *mat_dot(int rows, int cols, const double *left, const double *right, double *result);
const double *mat_scale(int rows, int cols, double mult, const double *mat, double *result);
const double *mat_mult(int lrows, int lcols, int rcols, const double *left, const double *right, double *result);

#endif
