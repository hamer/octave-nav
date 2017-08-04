#include "matrix.h"

const double *mat_add(int rows, int cols, const double *left, const double *right, double *result) {
    int num = rows * cols;
    double *p = result;

    while (num--)
        *p++ = *left++ + *right++;

    return result;
}

const double *mat_sub(int rows, int cols, const double *left, const double *right, double *result) {
    int num = rows * cols;
    double *p = result;

    while (num--)
        *p++ = *left++ - *right++;

    return result;
}

const double *mat_dot(int rows, int cols, const double *left, const double *right, double *result) {
    int num = rows * cols;
    double *p = result;

    while (num--)
        *p++ = *left++ * *right++;

    return result;
}

const double *mat_scale(int rows, int cols, double fact, const double *mat, double *result) {
    int num = rows * cols;
    double *p = result;

    while (num--)
        *p++ = fact * *mat++;

    return result;
}

const double *mat_transpose(int rows, int cols, const double *mat, double *result) {
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            result[i + j * rows] = *mat++;

    return result;
}

const double *mat_mult(int lrows, int lcols, int rcols, const double *left, const double *right, double *result) {
    double *p = result;

    for (int i = 0; i < lrows; ++i)
        for (int j = 0; j < rcols; ++j) {
            *p = 0;

            for (int k = 0; k < lcols; ++k)
                *p += left[k + i * lcols] * right[j + k * rcols];

            ++p;
        }

    return result;
}
