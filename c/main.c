#include <stdio.h>
#include <math.h>

#include "vector.h"
#include "matrix.h"
#include "quaternion.h"
#include "rotation.h"
#include "geod.h"

static const double deg2rad = M_PI / 180.0;
static const double rad2deg = 180.0 / M_PI;

static const double *mat_print(int rows, int cols, const double *data, const char *desc);

void check_mult(void);
void check_convertions(void);
void check_geod(void);
void check_rot(void);
void check_slerp(void);

int main(void) {
    check_convertions();
    check_mult();
    check_geod();
    check_rot();
    check_slerp();

    return 0;
}

static const double *mat_print(int rows, int cols, const double *data, const char *desc) {
    const double *p = data;
    printf("%s:\n", desc);

    for (int i = 0; i < rows; ++i) {
        printf("    [ ");

        for (int j = 0; j < cols; ++j)
            printf("%+9.4f ", *p++);

        printf("]\n");
    }

    printf("\n");
    return data;
}

void check_convertions(void) {
    double rpy[3] = { 1.5, 1.5, -1.2 };
    double dcm[9], quat[4], dcm_rs[9], rpy_rs[3];
    double quat2[4], rpy2[4];

    rpy2dcm(rpy, dcm);
    dcm2quat(dcm, quat);
    quat2dcm(quat, dcm_rs);
    dcm2rpy(dcm_rs, rpy_rs);
    rpy2quat(rpy, quat2);
    quat2rpy(quat2, rpy2);

    printf("===== RPY <-> DCM <-> QUAT convertions test:\n");
    mat_print(1, 3, rpy, "RPY(S)");
    mat_print(1, 3, rpy_rs, "RPY(D)");
    mat_print(1, 3, rpy2, "RPY(Q)");
    mat_print(3, 3, dcm, "DCM(S)");
    mat_print(3, 3, dcm_rs, "DCM(D)");
    mat_print(1, 4, quat, "QUAT(D)");
    mat_print(1, 4, quat2, "QUAT(R)");
}

void check_mult(void) {
    double v1[3] = { 1.0, 2.0, 3.0 }, v2[3] = { 5.0, 6.0, 8.0 };
    double m1[9] = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
    double m2[9] = { 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 1.0, 0.0, 1.0 };
    double q1[4] = { 0.1, 0.2, 0.3, 0.4 }, q2[4] = { 0.5, 0.6, 0.7, 0.8 };
    double v_rs[3], m_rs[9], q_rs[4];

    vec_cross(v1, v2, v_rs);
    mat_mult(3, 3, 3, m1, m2, m_rs);
    quat_mult(q1, q2, q_rs);

    printf("===== Multiplication test\n");
    mat_print(1, 3, v1, "VEC(1)");
    mat_print(1, 3, v2, "VEC(2)");
    mat_print(1, 3, v_rs, "VEC(R)");
    mat_print(3, 3, m1, "MAT(1)");
    mat_print(3, 3, m2, "MAT(2)");
    mat_print(3, 3, m_rs, "MAT(R)");
    mat_print(1, 4, q1, "QUAT(1)");
    mat_print(1, 4, q2, "QUAT(2)");
    mat_print(1, 4, q_rs, "QUAT(R)");
}

void check_geod(void) {
    double geod[3] = { 50.0, 13.0, 45.0 }, ecef[3], geod_rs[3];
    printf("===== RPY <-> DCM <-> QUAT convertions test:\n");

    geod2ecef(geod, ecef);
    ecef2geod(ecef, geod_rs);

    mat_print(1, 3, geod, "GEOD(S)");
    mat_print(1, 3, geod_rs, "GEOD(D)");
    mat_print(1, 3, ecef, "ECEF");

    for (int i = -6; i <= 6; ++i) {
        geod[0] = 15.0 * i;

        geod2ecef(geod, ecef);
        ecef2geod(ecef, geod_rs);

        mat_print(1, 3, geod, "GEOD(S)");
        mat_print(1, 3, geod_rs, "GEOD(D)");
        mat_print(1, 3, ecef, "ECEF");
    }
}

void check_rot(void) {
    double rpy[3] = { 45.0, -180.0, 90.0 }, xyz[3] = { 10.0, 20.0, 30.0 };
    double dcm[9], quat[4];
    double d_rs[3], q_rs[3];

    printf("===== Checking rotation\n");
    mat_print(1, 3, xyz, "XYZ");
    mat_print(1, 3, rpy, "RPY");

    vec_scale(3, deg2rad, rpy, rpy);
    mat_mult(3, 3, 1, rpy2dcm(rpy, dcm), xyz, d_rs);
    quat_rot(xyz, rpy2quat(rpy, quat), q_rs);

    mat_print(1, 3, d_rs, "XYZ(D)");
    mat_print(1, 3, q_rs, "XYZ(Q)");
}

void check_slerp(void) {
    double rpy_a[3] = { 0.0, 0.0, 0.0 }, rpy_b[3] = { 0.0, 10.0, 180.0 };
    double quat_a[4], quat_b[4], quat_rs[4], rpy_rs[3];
    char text[] = "RPY(D)";

    printf("===== Checking SLERP\n");
    mat_print(1, 3, rpy_a, "RPY(A)");
    mat_print(1, 3, rpy_b, "RPY(B)");

    rpy2quat(vec_scale(3, deg2rad, rpy_a, rpy_a), quat_a);
    rpy2quat(vec_scale(3, deg2rad, rpy_b, rpy_b), quat_b);

    for (int i = 0; i <= 10; ++i) {
        quat_slerp(0.1 * i, quat_a, quat_b, quat_rs);
        vec_scale(3, rad2deg, quat2rpy(quat_rs, rpy_rs), rpy_rs);

        text[4] = '0' + i;
        mat_print(1, 3, rpy_rs, text);
    }
}
