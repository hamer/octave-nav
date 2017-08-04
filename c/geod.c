#include <math.h>
#include "geod.h"

static const double Sa  = 6378137.0;
static const double Sb  = 6356752.3142;
static const double Se  = 0.0818191909289069;     // e = sqrt(1 - (b/a)^2)
static const double Se2 = 0.00673949675658700;    // e^2 / (1 - e^2)
//static const double Sf  = 1.0 / 298.257223563;    // f = 1 - b/a

static const double deg2rad = M_PI / 180.0;
static const double rad2deg = 180.0 / M_PI;

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
