#ifndef __GEOD_H__
#define __GEOD_H__

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

#endif
