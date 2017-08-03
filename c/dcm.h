#ifndef __DCM_H__
#define __DCM_H__

void rpy2dcm(const double *rpy, double *dcm);
void dcm2rpy(const double *dcm, double *rpy);

void rpy2quat(const double *rpy, double *quat);
void quat2rpy(const double *quat, double *rpy);

void dcm2quat(const double *dcm, double *quat);
void quat2dcm(const double *quat, double *dcm);

#endif
