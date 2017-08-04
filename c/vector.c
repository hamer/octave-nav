#include <math.h>
#include "matrix.h"

#include "vector.h"

double vec_norm(int len, const double *vec) {
    return sqrt(vec_dot(len, vec, vec));
}

double vec_dot(int len, const double *left, const double *right) {
    double result = 0.0;

    while (len--)
        result += *left++ * *right++;

    return result;
}


const double *vec_add(int len, const double *left, const double *right, double *result) {
    return mat_add(len, 1, left, right, result);
}

const double *vec_sub(int len, const double *left, const double *right, double *result) {
    return mat_sub(len, 1, left, right, result);
}

const double *vec_scale(int len, double fact, const double *vec, double *result) {
    return mat_scale(len, 1, fact, vec, result);
}

const double *vec_cross(const double *left, const double *right, double *result) {
    result[0] = left[1] * right[2] - left[2] * right[1];
    result[1] = left[2] * right[0] - left[0] * right[2];
    result[2] = left[0] * right[1] - left[1] * right[0];
    return result;
}

