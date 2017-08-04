#include "vector.h"

#include "quaternion.h"

const double *quat_norm(const double *quat, double *result) {
    double norm = vec_norm(4, quat);

    // if norm is 0.0, just copy quat to result
    // TODO: is this correct to compare IEEE754 value with 0.0 to prevent FPE?
    vec_scale(4, norm != 0.0 ? 1.0 / norm : 1.0, quat, result);

    return result;
}

const double *quat_conj(const double *quat, double *result) {
    result[0] =  quat[0];
    result[1] = -quat[1];
    result[2] = -quat[2];
    result[3] = -quat[3];

    return result;
}

const double *quat_mult(const double *left, const double *right, double *result) {
    result[0] = left[0] * right[0] - left[1] * right[1] - left[2] * right[2] - left[3] * right[3];
    result[1] = left[1] * right[0] + left[0] * right[1] - left[3] * right[2] + left[2] * right[3];
    result[2] = left[2] * right[0] + left[3] * right[1] + left[0] * right[2] - left[1] * right[3];
    result[3] = left[3] * right[0] - left[2] * right[1] + left[1] * right[2] + left[0] * right[3];

    return result;
}
