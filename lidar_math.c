#include "lidar.h"

static bool on_seg(Punto start, Punto end, double x, double y, double tol) {
    double minx = fmin(start.x, end.x) - tol;
    double maxx = fmax(start.x, end.x) + tol;
    double miny = fmin(start.y, end.y) - tol;
    double maxy = fmax(start.y, end.y) + tol;
    if (x < minx || x > maxx || y < miny || y > maxy) return false;
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double lsq = dx * dx + dy * dy;
    if (lsq < 1e-9) return false;
    double t = ((x - start.x) * dx + (y - start.y) * dy) / lsq;
    t = fmax(0.0, fmin(1.0, t));
    double px = start.x + t * dx;
    double py = start.y + t * dy;
    double d = hypot(x - px, y - py);
    return d < tol;
}
// Kesisim hesaplamak part

int calc_intersect(const Retta lines[], int nL, double rmax, Inter ints[], int *closest) {
    int n = 0;
    double mindist = 1e9;
    *closest = -1;
    for (int i = 0; i < nL; i++) {
        for (int j = i + 1; j < nL; j++) {
            double dotc = fabs(lines[i].a * lines[j].a + lines[i].b * lines[j].b);
            if (dotc > 0.995) continue;
            double a1 = lines[i].a, b1 = lines[i].b, c1 = lines[i].c;
            double a2 = lines[j].a, b2 = lines[j].b, c2 = lines[j].c;
            double det = a1 * b2 - a2 * b1;
            if (fabs(det) < 1e-12) continue;
            double x = (b1 * c2 - b2 * c1) / det;
            double y = (c1 * a2 - c2 * a1) / det;
            double exr = 0.30; 
            double dx1 = lines[i].p_end.x - lines[i].p_start.x;
            double dy1 = lines[i].p_end.y - lines[i].p_start.y;
            double len1 = hypot(dx1, dy1);
            if (len1 < 0.01) continue;
            dx1 /= len1; dy1 /= len1;
            double ex1 = len1 * exr;
            Punto s1s = {lines[i].p_start.x - dx1 * ex1, lines[i].p_start.y - dy1 * ex1};
            Punto s1e = {lines[i].p_end.x + dx1 * ex1, lines[i].p_end.y + dy1 * ex1};
            double dx2 = lines[j].p_end.x - lines[j].p_start.x;
            double dy2 = lines[j].p_end.y - lines[j].p_start.y;
            double len2 = hypot(dx2, dy2);
            if (len2 < 0.01) continue;
            dx2 /= len2; dy2 /= len2;
            double ex2 = len2 * exr;
            Punto s2s = {lines[j].p_start.x - dx2 * ex2, lines[j].p_start.y - dy2 * ex2};
            Punto s2e = {lines[j].p_end.x + dx2 * ex2, lines[j].p_end.y + dy2 * ex2};

            double tol = 0.35;
            bool on1 = on_seg(s1s, s1e, x, y, tol);
            bool on2 = on_seg(s2s, s2e, x, y, tol);
            if (!on1 || !on2) continue;
            double minep = 1e9;
            double eps[4][2] = {
                {lines[i].p_start.x, lines[i].p_start.y},
                {lines[i].p_end.x, lines[i].p_end.y},
                {lines[j].p_start.x, lines[j].p_start.y},
                {lines[j].p_end.x, lines[j].p_end.y}};
            for (int k = 0; k < 4; k++) {
                double d = hypot(x - eps[k][0], y - eps[k][1]);
                if (d < minep) minep = d;}
            if (minep > 0.35) continue;   
            if (fabs(x) > rmax * 1.5 || fabs(y) > rmax * 1.5) continue;
            double dota = (lines[i].a * lines[j].a + lines[i].b * lines[j].b);
            double lena = hypot(lines[i].a, lines[i].b);
            double lenb = hypot(lines[j].a, lines[j].b);
            if (lena < 1e-9 || lenb < 1e-9) continue;
            double cosa = dota / (lena * lenb);
            cosa = fmin(1.0, fmax(-1.0, cosa));
            double ang = acos(fabs(cosa)) * 180.0 / M_PI;
            if (ang < 60.0 || ang > 120.0) continue; // 30derece kulandisek daha iyi cekiyor
            double dist = hypot(x, y);
            

            ints[n].x = x;
            ints[n].y = y;
            ints[n].ang = ang;
            ints[n].dist = dist;
            ints[n].idx1 = i;
            ints[n].idx2 = j;
            ints[n].ok = true;
            if (dist < mindist && dist > 0.1) {
                mindist = dist;
                *closest = n;}
            n++;}}
    return n;
}
