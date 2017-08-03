#include <stdio.h>
#include <inttypes.h>

#include "matrix.h"
#include "dcm.h"
#include "geod.h"

int main(void) {
    double rpy[3] = { 1.5, 1.5, -1.2 };
    double dcm[9], quat[4], dcm_rs[9], rpy_rs[3];
    double quat2[4], rpy2[4];

    rpy2dcm(rpy, dcm);
    dcm2quat(dcm, quat);
    quat2dcm(quat, dcm_rs);
    dcm2rpy(dcm_rs, rpy_rs);
    rpy2quat(rpy, quat2);
    quat2rpy(quat2, rpy2);

    mat_print(1, 3, rpy, "RPY(S)");
    mat_print(1, 3, rpy_rs, "RPY(D)");
    mat_print(1, 3, rpy2, "RPY(Q)");
    mat_print(3, 3, dcm, "DCM(S)");
    mat_print(3, 3, dcm_rs, "DCM(D)");
    mat_print(1, 4, quat, "QUAT(D)");
    mat_print(1, 4, quat2, "QUAT(R)");

    return 0;
}

