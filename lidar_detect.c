#include "lidar.h"

// The RANSAC algoritm
int detect_ransac(const Punto pts[], int n, Retta lines[]) {
    if (n < MIN_PTS) return 0;
    int nl = 0;
    bool used[MAX_RAGGI] = {false};
    srand(42);
    
    for (int iter = 0; iter < 500 && nl < MAX_RETTE; iter++) {
        int i1 = -1, i2 = -1;
        for (int att = 0; att < 100; att++) {
            i1 = rand() % n;
            if (!used[i1]) break;}
        if (used[i1]) continue;
        for (int att = 0; att < 100; att++) {
            i2 = rand() % n;
            if (i2 != i1 && !used[i2]) break; }
        if (used[i2] || i2 == i1) continue;
        
        double dx = pts[i2].x - pts[i1].x;
        double dy = pts[i2].y - pts[i1].y;
        double len = hypot(dx, dy);
        if (len < 0.1 || len > 3.0) continue;
        double a = -dy / len;
        double b = dx / len;
        double c = -(a * pts[i1].x + b * pts[i1].y);
        int inliers[MAX_RAGGI];
        int nin = 0;
        for (int i = 0; i < n; i++) {
            if (used[i]) continue;
            double d = fabs(a * pts[i].x + b * pts[i].y + c);
            if (d < THRESH) inliers[nin++] = i;}
        if (nin < MIN_PTS) continue;
        
        double vx = dx / len, vy = dy / len;
        double proj[MAX_RAGGI];
        for (int i = 0; i < nin; i++) {
            proj[i] = vx * pts[inliers[i]].x + vy * pts[inliers[i]].y;
        }
        for (int i = 0; i < nin - 1; i++) {
            for (int j = 0; j < nin - i - 1; j++) {
                if (proj[j] > proj[j+1]) {
                    double tmp = proj[j]; proj[j] = proj[j+1]; proj[j+1] = tmp;
                    int ti = inliers[j]; inliers[j] = inliers[j+1]; inliers[j+1] = ti;}}}
                    int bs = 0, bl = 1, cs = 0, cl = 1;
                    for (int i = 1; i < nin; i++) {
                    if (proj[i] - proj[i-1] < 0.15) {
                        cl++; } else {
                    if (cl > bl) { bl = cl; bs = cs; }
                        cs = i; cl = 1;}}
        if (cl > bl) { bl = cl; bs = cs; }
        if (bl < MIN_PTS) continue;
         int si = inliers[bs];
        int ei = inliers[bs + bl - 1];
        double llen = hypot(pts[ei].x - pts[si].x, pts[ei].y - pts[si].y);
        if (llen < 0.25) continue;
        lines[nl].a = a;
        lines[nl].b = b;
        lines[nl].c = c;
        lines[nl].idx_start = si;
        lines[nl].idx_end = ei;
        lines[nl].p_start = pts[si];
        lines[nl].p_end = pts[ei];
        for (int i = 0; i < bl; i++) used[inliers[bs + i]] = true;
        nl++;
    }
    return nl;
}

// PCA algoritm
int detect_pca(const Punto pts[], int n, bool used[], Retta lines[], int max) {
    if (n < MIN_PTS) return 0;
    int nl = 0;
    
    for (int iter = 0; iter < 100 && nl < max; iter++) {
        int seed = -1;
        for (int i = 0; i < n; i++) {
            if (!used[i]) { seed = i; break; }
        }
        if (seed == -1) break;
        
        int clust[MAX_RAGGI];
        int nc = 0;
        clust[nc++] = seed;
        used[seed] = true;
        
        for (int i = 0; i < n; i++) {
            if (used[i]) continue;
            bool is_near = false;  // near is reserved ahahhahahh
            for (int j = 0; j < nc; j++) {
                double d = hypot(pts[i].x - pts[clust[j]].x, pts[i].y - pts[clust[j]].y);
                if (d < 0.2) { is_near = true; break; }
            }
            if (is_near) {
                clust[nc++] = i;
                used[i] = true;
            }
        }
        if (nc < MIN_PTS) {
            for (int i = 0; i < nc; i++) used[clust[i]] = false;
            continue;}
         double mx = 0, my = 0;
        for (int i = 0; i < nc; i++) { mx += pts[clust[i]].x;
            my += pts[clust[i]].y;}
        mx /= nc; my /= nc;  
        double sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < nc; i++) {
            double dx = pts[clust[i]].x - mx;
            double dy = pts[clust[i]].y - my;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
    }
        
        double tr = sxx + syy;
        double det = sxx * syy - sxy * sxy;
        if (tr < 1e-9) continue;
        double disc = tr * tr / 4 - det;
        if (disc < 0) disc = 0;
        double lambda = tr / 2 + sqrt(disc);
        

        double vx, vy;
        if (fabs(sxy) > 1e-9) {
            vx = lambda - syy;
            vy = sxy;
        } else {
            vx = (sxx > syy) ? 1.0 : 0.0;
            vy = (sxx > syy) ? 0.0 : 1.0;
        }

        double norm = hypot(vx, vy);
        if (norm < 1e-9) continue;
        vx /= norm; vy /= norm;
        
        double a = -vy;
        double b = vx;
        double c = -(a * mx + b * my);
        
        double minp = 1e9, maxp = -1e9;
        int mini = 0, maxi = 0;
        for (int i = 0; i < nc; i++) {
            double p = vx * pts[clust[i]].x + vy * pts[clust[i]].y;
            if (p < minp) { minp = p; mini = clust[i]; }
            if (p > maxp) { maxp = p; maxi = clust[i]; }
        }
        
        double llen = hypot(pts[maxi].x - pts[mini].x, pts[maxi].y - pts[mini].y);
        if (llen < 0.25) continue;
        
        lines[nl].a = a;
        lines[nl].b = b;
        lines[nl].c = c;
        lines[nl].idx_start = mini;
        lines[nl].idx_end = maxi;
        lines[nl].p_start = pts[mini];
        lines[nl].p_end = pts[maxi];
        nl++;
    }
    return nl;
}

// dogru tespitileme suan RANSAC  ve PCA  berlisme 
int detect_lines(const Punto pts[], int n, Retta lines[]) {
    if (n < MIN_PTS) return 0;
    
    // RANSAC
    int nr = detect_ransac(pts, n, lines);
    printf("[RANSAC] ile %d dogru\n", nr);
    bool used[MAX_RAGGI] = {false};
    for (int i = 0; i < nr; i++) {
        for (int j = 0; j < n; j++) {
            double d = fabs(lines[i].a * pts[j].x + lines[i].b * pts[j].y + lines[i].c);
            if (d < THRESH * 2) used[j] = true;
    }
    }
    
    //PCA
    if (nr < MAX_RETTE - 2) {
        Retta plines[MAX_RETTE];
        int np = detect_pca(pts, n, used, plines, MAX_RETTE - nr);
        printf("[PCA] ile %d ek dogru\n", np);
        
        for (int i = 0; i < np && nr + i < MAX_RETTE; i++) {
            bool dup = false;
            for (int j = 0; j < nr; j++) {
                double dot = fabs(lines[j].a * plines[i].a + lines[j].b * plines[i].b);
                double dist = fabs(lines[j].a * plines[i].p_start.x + 
                                 lines[j].b * plines[i].p_start.y + lines[j].c);
                if (dot > 0.98 && dist < 0.1) { dup = true; break; }}
            if (!dup) lines[nr++] = plines[i];}}
    return nr;
}
