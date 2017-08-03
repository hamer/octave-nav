#include <math.h>
#include "dcm.h"

void rpy2dcm(const double *rpy, double *dcm) {
    const double cr = cos(rpy[0]), cp = cos(rpy[1]), cy = cos(rpy[2]);
    const double sr = sin(rpy[0]), sp = sin(rpy[1]), sy = sin(rpy[2]);

    dcm[0] = cp * cy;
    dcm[1] = sr * sp * cy - cr * sy;
    dcm[2] = cr * sp * cy + sr * sy;

    dcm[3] = cp * sy;
    dcm[4] = sr * sp * sy + cr * cy;
    dcm[5] = cr * sp * sy - sr * cy;

    dcm[6] = -sp;
    dcm[7] = sr * cp;
    dcm[8] = cr * cp;
}

void dcm2rpy(const double *dcm, double *rpy) {
    rpy[0] = atan2(dcm[7], dcm[8]);
    rpy[1] = asin(-dcm[6]);
    rpy[2] = atan2(dcm[3], dcm[0]);
}

void rpy2quat(const double *rpy, double *quat) {
    const double cr = cos(rpy[0] / 2.0), cp = cos(rpy[1] / 2.0), cy = cos(rpy[2] / 2.0);
    const double sr = sin(rpy[0] / 2.0), sp = sin(rpy[1] / 2.0), sy = sin(rpy[2] / 2.0);

    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;
}

void quat2rpy(const double *quat, double *rpy) {
    rpy[0] = atan2(2.0 * (quat[2] * quat[3] + quat[1] * quat[0]), // dcm[7]
            1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])); // dcm[8]
    rpy[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1])); // -dcm[6]
    rpy[2] = atan2(2.0 * (quat[1] * quat[2] + quat[3] * quat[0]), // dcm[3]
            1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3])); // dcm[0]
}

void dcm2quat(const double *dcm, double *quat) {
    const double m[4] = {
        1.0 + dcm[0] + dcm[4] + dcm[8],
        1.0 + dcm[0] - dcm[4] - dcm[8],
        1.0 - dcm[0] + dcm[4] - dcm[8],
        1.0 - dcm[0] - dcm[4] + dcm[8]
    };

    int mi = 0;
    double mv = m[mi];

    for (int i = 1; i < 4; ++i)
        if (m[i] > mv)
            mv = m[mi = i];

    switch (mi) {
        case 0:
            quat[0] = sqrt(0.25 * mv);
            quat[1] = 0.25 * (dcm[7] - dcm[5]) / quat[0];
            quat[2] = 0.25 * (dcm[2] - dcm[6]) / quat[0];
            quat[3] = 0.25 * (dcm[3] - dcm[1]) / quat[0];
            break;

        case 1:
            quat[1] = sqrt(0.25 * mv);
            quat[0] = 0.25 * (dcm[7] - dcm[5]) / quat[1];
            quat[2] = 0.25 * (dcm[3] + dcm[1]) / quat[1];
            quat[3] = 0.25 * (dcm[2] + dcm[6]) / quat[1];
            break;

        case 2:
            quat[2] = sqrt(0.25 * mv);
            quat[0] = 0.25 * (dcm[2] - dcm[6]) / quat[2];
            quat[1] = 0.25 * (dcm[3] + dcm[1]) / quat[2];
            quat[3] = 0.25 * (dcm[7] + dcm[5]) / quat[2];
            break;

        case 3:
            quat[3] = sqrt(0.25 * mv);
            quat[0] = 0.25 * (dcm[3] - dcm[1]) / quat[3];
            quat[1] = 0.25 * (dcm[2] + dcm[6]) / quat[3];
            quat[2] = 0.25 * (dcm[7] + dcm[5]) / quat[3];
            break;
    }
}

void quat2dcm(const double *quat, double *dcm) {
    dcm[0] = 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]);
    dcm[4] = 1.0 - 2.0 * (quat[1] * quat[1] + quat[3] * quat[3]);
    dcm[8] = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]);

    dcm[1] = 2.0 * (quat[1] * quat[2] - quat[3] * quat[0]);
    dcm[2] = 2.0 * (quat[3] * quat[1] + quat[2] * quat[0]);
    dcm[3] = 2.0 * (quat[1] * quat[2] + quat[3] * quat[0]);
    dcm[5] = 2.0 * (quat[2] * quat[3] - quat[1] * quat[0]);
    dcm[6] = 2.0 * (quat[3] * quat[1] - quat[2] * quat[0]);
    dcm[7] = 2.0 * (quat[2] * quat[3] + quat[1] * quat[0]);
}
