#include <math.h>
#include "vector.h"
#include "matrix.h"
#include "rotation.h"

#include "geod.h"

static const double Sa  = 6378137.0;
static const double Sb  = 6356752.3142;
static const double Se  = 0.0818191909289069;     // e = sqrt(1 - (b/a)^2)
static const double Se2 = 0.00673949675658700;    // e^2 / (1 - e^2)
//static const double Sf  = 1.0 / 298.257223563;    // f = 1 - b/a

const double deg2rad = M_PI / 180.0;
const double rad2deg = 180.0 / M_PI;

static double get_n(double phi);

static double get_n(double phi) {
    return Sa / sqrt(1.0 - pow((Se * sin(phi)), 2.0));
}

const double *geod2ecef(const double *geod, double *ecef) {
    const double cp = cos(geod[0] * deg2rad), cl = cos(geod[1] * deg2rad);
    const double sp = sin(geod[0] * deg2rad), sl = sin(geod[1] * deg2rad);
    const double n = get_n(geod[0] * deg2rad);
    const double r = (n + geod[2]) * cp;

    ecef[0] = r * cl;
    ecef[1] = r * sl;
    ecef[2] = (Sb / Sa * Sb / Sa * n + geod[2]) * sp;

    return ecef;
}

const double *ecef2geod(const double *ecef, double *geod) {
    const double p = hypot(ecef[0], ecef[1]);
    const double q = atan2(ecef[2] * Sa, p * Sb);
    const double phi = atan2(ecef[2] + Se2 * Sb * pow(sin(q), 3.0), p - Sa * Se * Se * pow(cos(q), 3.0));
    const double h = p / cos(phi) - get_n(phi);

    geod[0] = phi * rad2deg;
    geod[1] = atan2(ecef[1], ecef[0]) * rad2deg;
    geod[2] = h;

    return geod;
}

const double *geod2dcm(const double *geod, double *dcm) {
    const double cp = cos(geod[0] * deg2rad), cl = cos(geod[1] * deg2rad);
    const double sp = sin(geod[0] * deg2rad), sl = sin(geod[1] * deg2rad);

    dcm[0] = -sl;   dcm[1] = -sp * cl;  dcm[2] =  cp * cl;
    dcm[3] =  cl;   dcm[4] = -sp * sl;  dcm[5] =  cp * sl;
    dcm[6] =  0;    dcm[7] =  cp;       dcm[8] =  sp;

    return dcm;
}

const double *geod2wmerc(const double *geod, double *wmerc) {
    wmerc[0] = Sa * geod[1] * deg2rad;
    wmerc[1] = Sa * log(tan(M_PI_4 + geod[0] * deg2rad / 2));
    wmerc[2] = geod[2];

    return wmerc;
}

const double *wmerc2geod(const double *wmerc, double *geod) {
    geod[0] = 90.0 - 2.0 * atan(exp(-wmerc[1] / Sa)) * rad2deg;
    geod[1] = wmerc[0] / Sa * rad2deg;
    geod[2] = wmerc[2];

    return geod;
}

double wmerc2scale(const double *wmerc) {
    return cosh(wmerc[1] / Sa);
}

double geod2scale(const double *geod) {
    return 1.0 / cos(geod[0] * deg2rad);
}

const double *enu2geod(const double *geod_src, const double *enu_tgt, double *result) {
    double enu_dcm[9], ecef_src[3], ecef_tgt[3];

    mat_mult(3, 3, 1, geod2dcm(geod_src, enu_dcm), enu_tgt, ecef_tgt);
    vec_add(3, geod2ecef(geod_src, ecef_src), ecef_tgt, ecef_tgt);
    return ecef2geod(ecef_tgt, result);
}

const double *ned2geod(const double *geod_src, const double *ned_tgt, double *result) {
    double enu_tgt[3] = { ned_tgt[1], ned_tgt[0], -ned_tgt[2] };
    return enu2geod(geod_src, enu_tgt, result);
}

const double *geod2enu(const double *geod_src, const double *geod_tgt, double *result) {
    double enu_dcm[9], ecef_src[3], tgt[3];

    vec_sub(3, geod2ecef(geod_tgt, tgt), geod2ecef(geod_src, ecef_src), tgt);
    geod2dcm(geod_src, enu_dcm);
    return mat_mult(3, 3, 1, dcm_inv(enu_dcm, enu_dcm), tgt, result);
}

const double *geod2ned(const double *geod_src, const double *geod_tgt, double *result) {
    double enu_tgt[3];
    geod2enu(geod_src, geod_tgt, enu_tgt);

    result[0] =  enu_tgt[1];
    result[1] =  enu_tgt[0];
    result[2] = -enu_tgt[2];
    return result;
}
