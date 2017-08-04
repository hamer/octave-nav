#ifndef __QUATERNION_H__
#define __QUATERNION_H__

const double *quat_norm(const double *quat, double *result);
const double *quat_conj(const double *quat, double *result);
const double *quat_mult(const double *left, const double *right, double *result);

#endif
