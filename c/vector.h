#ifndef __VECTOR_H__
#define __VECTOR_H__

double vec_norm(int len, const double *vec);
double vec_dot(int len, const double *left, const double *right);
const double *vec_add(int len, const double *left, const double *right, double *result);
const double *vec_sub(int len, const double *left, const double *right, double *result);
const double *vec_scale(int len, double fact, const double *vec, double *result);
const double *vec_cross(const double *left, const double *right, double *result);

#endif
