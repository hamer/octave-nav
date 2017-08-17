#ifndef __ROTATION_H__
#define __ROTATION_H__

#define wrap_180(x) wrap_odd(x,  360.0) // wrap to (-180.0, 180.0]
#define wrap_360(x) wrap_even(x, 360.0) // wrap to [0.0, 360.0)

double smod(double x, double mod);
double wrap_odd(double x, double range);
double wrap_even(double x, double range);

const double *rpy2dcm(const double *rpy, double *dcm);
const double *dcm2rpy(const double *dcm, double *rpy);

const double *rpy2quat(const double *rpy, double *quat);
const double *quat2rpy(const double *quat, double *rpy);

const double *dcm2quat(const double *dcm, double *quat);
const double *quat2dcm(const double *quat, double *dcm);

const double *quat_rot(const double *xyz, const double *quat, double *rs);
const double *quat_slerp(double k, const double *left, const double *right, double *result);

const double *dcm_inv(const double *dcm, double *result);

#endif
