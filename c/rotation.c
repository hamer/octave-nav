#include <math.h>
#include "vector.h"
#include "matrix.h"
#include "quaternion.h"

#include "rotation.h"

double smod(double x, double mod) {
    return x - (double)floor(x / mod + 0.5) * mod;
}

double wrap_odd(double x, double range) {
    return -smod(-x, range);
}

double wrap_even(double x, double range) {
    return smod(x - range / 2.0, range) + range / 2.0;
}

const double *rpy2dcm(const double *rpy, double *dcm) {
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

    return dcm;
}

const double *dcm2rpy(const double *dcm, double *rpy) {
    rpy[0] = atan2(dcm[7], dcm[8]);
    rpy[1] = asin(-dcm[6]);
    rpy[2] = atan2(dcm[3], dcm[0]);

    return rpy;
}

const double *rpy2quat(const double *rpy, double *quat) {
    const double cr = cos(rpy[0] / 2.0), cp = cos(rpy[1] / 2.0), cy = cos(rpy[2] / 2.0);
    const double sr = sin(rpy[0] / 2.0), sp = sin(rpy[1] / 2.0), sy = sin(rpy[2] / 2.0);

    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;

    return quat;
}

const double *quat2rpy(const double *quat, double *rpy) {
    rpy[0] = atan2(2.0 * (quat[2] * quat[3] + quat[1] * quat[0]),  //  dcm[7]
             1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])); //  dcm[8]
    rpy[1] = asin( 2.0 * (quat[2] * quat[0] - quat[3] * quat[1])); // -dcm[6]
    rpy[2] = atan2(2.0 * (quat[1] * quat[2] + quat[3] * quat[0]),  //  dcm[3]
             1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3])); //  dcm[0]

    return rpy;
}

const double *dcm2quat(const double *dcm, double *quat) {
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
            quat[3] = 0.25 * (dcm[2] + dcm[6]) / quat[1];
            quat[2] = 0.25 * (dcm[3] + dcm[1]) / quat[1];
            break;

        case 2:
            quat[2] = sqrt(0.25 * mv);
            quat[3] = 0.25 * (dcm[7] + dcm[5]) / quat[2];
            quat[0] = 0.25 * (dcm[2] - dcm[6]) / quat[2];
            quat[1] = 0.25 * (dcm[3] + dcm[1]) / quat[2];
            break;

        case 3:
            quat[3] = sqrt(0.25 * mv);
            quat[2] = 0.25 * (dcm[7] + dcm[5]) / quat[3];
            quat[1] = 0.25 * (dcm[2] + dcm[6]) / quat[3];
            quat[0] = 0.25 * (dcm[3] - dcm[1]) / quat[3];
            break;
    }

    return quat;
}

const double *quat2dcm(const double *quat, double *dcm) {
    dcm[0] = 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]);
    dcm[4] = 1.0 - 2.0 * (quat[1] * quat[1] + quat[3] * quat[3]);
    dcm[8] = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]);

    dcm[1] = 2.0 * (quat[1] * quat[2] - quat[3] * quat[0]);
    dcm[3] = 2.0 * (quat[1] * quat[2] + quat[3] * quat[0]);
    dcm[2] = 2.0 * (quat[3] * quat[1] + quat[2] * quat[0]);
    dcm[6] = 2.0 * (quat[3] * quat[1] - quat[2] * quat[0]);
    dcm[5] = 2.0 * (quat[2] * quat[3] - quat[1] * quat[0]);
    dcm[7] = 2.0 * (quat[2] * quat[3] + quat[1] * quat[0]);

    return dcm;
}

const double *quat_rot(const double *xyz, const double *quat, double *result) {
    double p[4] = { 0.0, xyz[0], xyz[1], xyz[2] };
    double tmp_a[4], tmp_b[4];

    // result = quat * p * conj(quat)
    quat_mult(p, quat_conj(quat, tmp_a), tmp_b);
    quat_mult(quat, tmp_b, tmp_a);

    result[0] = tmp_a[1];
    result[1] = tmp_a[2];
    result[2] = tmp_a[3];
    return result;
}

const double *quat_slerp(double k, const double *left, const double *right, double *result) {
    double dot = vec_dot(4, left, right);

    if (fabs(dot) > 0.9995) {
        // result = norm(left + t * (right - left))
        vec_scale(4, k, vec_sub(4, right, left, result), result);
        quat_norm(vec_add(4, left, result, result), result);
    } else {
        double cleft[4];

        vec_scale(4, dot >= 0.0 ? 1.0 : -1.0, left, cleft);
        dot = fabs(dot);

        double theta = k * acos(fmin(1.0, fmax(-1.0, dot)));

        // mbase = norm(right - dot * left)
        quat_norm(vec_sub(4, right, vec_scale(4, dot, cleft, result), result), result);

        // result = left * cos(theta) + mbase * sin(theta)
        vec_scale(4, cos(theta), cleft, cleft);
        vec_scale(4, sin(theta), result, result);
        vec_add(4, cleft, result, result);
    }

    return result;
}

const double *dcm_inv(const double *dcm, double *result) {
    double tmp;

    if (result != dcm) {
        result[0] = dcm[0];
        result[4] = dcm[4];
        result[8] = dcm[8];
    }

    tmp = dcm[1]; result[1] = dcm[3]; result[3] = tmp;
    tmp = dcm[2]; result[2] = dcm[6]; result[6] = tmp;
    tmp = dcm[5]; result[5] = dcm[7]; result[7] = tmp;

    return result;
}
