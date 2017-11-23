#ifndef __GEOD_H__
#define __GEOD_H__

#define GRAV_RADIUS(grav)   ((grav)[0]) // [m]
#define GRAV_LATITUDE(grav) ((grav)[1]) // [rad]
#define GRAV_GRAVITY(grav)  ((grav)[2]) // [m/sec^3]

extern const double Sa;     // semi-major axis
extern const double Sb;     // semi-minor axis
extern const double Sif;    // inverse flattening: if = 1/f; f = 1 - b/a
extern const double See;    // square of excentricity: ee = e^2; e = sqrt(1 - (b/a)^2)
extern const double Se2;    // seconf eccentricity: e2 = ee / (1 - ee)
extern const double SGM;    // GM [m^3/s^2]

extern const double deg2rad;
extern const double rad2deg;

const double *geod2ecef(const double *geod, double *ecef);
const double *ecef2geod(const double *ecef, double *geod);

const double *geod2dcm(const double *geod, double *dcm);

const double *geod2wmerc(const double *geod, double *wmerc);
const double *wmerc2geod(const double *wmerc, double *geod);

double wmerc2scale(const double *wmerc);
double geod2scale(const double *geod);

const double *enu2geod(const double *geod_src, const double *enu_tgt, double *result);
const double *ned2geod(const double *geod_src, const double *ned_tgt, double *result);
const double *geod2enu(const double *geod_src, const double *geod_tgt, double *result);
const double *geod2ned(const double *geod_src, const double *geod_tgt, double *result);

double lat2grav(double phi, double *grav);

#endif
