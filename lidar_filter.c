#include "lidar.h"

int filter_pts(const double ranges[], int cnt, double rmin, double rmax,
               double amin, double ainc, double amax,
               double fr[], double fa[], Punto fp[]) {
    int n = 0;
    for (int i = 0; i < cnt; i++) {
        double val = ranges[i];
        double ang = amin + ainc * i;
        if (isnan(val) || val <= 0.0 || val == -1.0) continue;
        if (val < rmin || val > rmax || val > 10.0) continue;
        double x = val * cos(ang);
        double y = val * sin(ang);
        if (fabs(x) < 0.05 && fabs(y) < 0.05) continue;
        fr[n] = val;
        fa[n] = ang;
        fp[n].x = x;
        fp[n].y = y;
        n++;
    }
    return n;
}
