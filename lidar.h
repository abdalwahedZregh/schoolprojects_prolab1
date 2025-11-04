#ifndef LIDAR_H
#define LIDAR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <windows.h>
#include <ctype.h>
#include <stdbool.h>
#include <curl/curl.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <time.h>

#define MAX_RAGGI 20000      // max nokta sayisi
#define MAX_RETTE 32         // max dogru sayisi
#define MIN_PTS 8            // min nokta 
#define THRESH 0.025         // esik degeri (2.5cm gibi)
#define PIX_SCALE 150.0      // piksel olcegi
#define WIN_W 950           
#define WIN_H 950            


typedef struct {
    char *buf;
    size_t size;
} MemData;

// x y koordinatlar
typedef struct {
    double x;  
    double y;  
} Punto;      //punto = nokta demek


typedef struct {
    double a, b, c;  //ax + by + c =0
    int idx_start, idx_end; 
    Punto p_start, p_end;
} Retta;  // dogru


typedef struct {
    double x, y;          
    double ang, dist;      
    int idx1, idx2;      
    bool ok;               
} Inter;   //  kesisim


void download(const char *url, const char *file);
void parse_toml(const char *file, double *amin, double *amax, double *ainc, 
                double *rmin, double *rmax, double ranges[], int *cnt);
void save_parsed(const char *out, double amin, double amax, double ainc, 
                 double rmin, double rmax, const double ranges[], int cnt);
bool load_parsed(const char *in, double *amin, double *amax, double *ainc, 
                 double *rmin, double *rmax, double ranges[], int *cnt);
int filter_pts(const double ranges[], int cnt, double rmin, double rmax,
               double amin, double ainc, double amax,
               double fr[], double fa[], Punto fp[]);
int detect_ransac(const Punto pts[], int n, Retta lines[]);
int detect_pca(const Punto pts[], int n, bool used[], Retta lines[], int max);
int detect_lines(const Punto pts[], int n, Retta lines[]);
int calc_intersect(const Retta lines[], int nL, double rmax, Inter ints[], int *closest);
bool draw_viz(const Punto pts[], int nP, const Retta lines[], int nL,
              const Inter ints[], int nI, int closest, double rmax);

#endif
