#ifndef __GEOD_H__
#define __GEOD_H__

void geod2ecef(const double *geod, double *ecef);
void ecef2geod(const double *ecef, double *geod);

void geod2dcm(const double *geod, double *dcm);

void geod2wmerc(const double *geod, double *wmerc);
void wmerc2geod(const double *wmerc, double *geod);

double wmerc2scale(const double *wmerc);
double geod2scale(const double *geod);

#endif
