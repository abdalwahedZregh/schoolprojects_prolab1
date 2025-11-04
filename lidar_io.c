#include "lidar.h"

size_t write_cb(void *data, size_t size, size_t nmemb, void *userp) {
    size_t total = size * nmemb;
    MemData *mem = (MemData *)userp;
    char *ptr = realloc(mem->buf, mem->size + total + 1);
    if (!ptr) return 0;
    mem->buf = ptr;
    memcpy(mem->buf + mem->size, data, total);
    mem->size += total;
    mem->buf[mem->size] = 0;
    return total;
}

void download(const char *url, const char *file) {
    CURL *curl = curl_easy_init();
    if (!curl) exit(1);
    MemData mem = {NULL, 0};
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &mem);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    
    if (curl_easy_perform(curl) != CURLE_OK || mem.size < 100) {
        curl_easy_cleanup(curl);
        free(mem.buf);
        exit(1);
    }
    FILE *f = fopen(file, "wb");
    if (f) {
        fwrite(mem.buf, 1, mem.size, f);
        fclose(f);
    }
    curl_easy_cleanup(curl);
    free(mem.buf);
}


void parse_toml(const char *file, double *amin, double *amax, double *ainc,
                double *rmin, double *rmax, double ranges[], int *cnt) {
    FILE *f = fopen(file, "r");
    if (!f) return;
    
    char line[65536];
    char rbuf[65536] = {0};
    bool in_scan = false, collecting = false;
    
    while (fgets(line, sizeof(line), f)) {
        char *s = line;
        while (isspace(*s)) s++;
        if (!*s || *s == '#') continue;
        
        if (*s == '[') {
            in_scan = (strncmp(s, "[scan]", 6) == 0);
            continue;
        }
        if (!in_scan) continue;
        
        if (strncmp(s, "angle_min", 9) == 0) *amin = strtod(strchr(s, '=') + 1, NULL);
        else if (strncmp(s, "angle_max", 9) == 0) *amax = strtod(strchr(s, '=') + 1, NULL);
        else if (strncmp(s, "angle_increment", 15) == 0) *ainc = strtod(strchr(s, '=') + 1, NULL);
        else if (strncmp(s, "range_min", 9) == 0) *rmin = strtod(strchr(s, '=') + 1, NULL);
        else if (strncmp(s, "range_max", 9) == 0) *rmax = strtod(strchr(s, '=') + 1, NULL);
        else if (strncmp(s, "ranges", 6) == 0) {
            collecting = true;
            const char *start = strchr(s, '[');
            if (start) {
                strncat(rbuf, start + 1, sizeof(rbuf) - strlen(rbuf) - 1);
                if (strchr(start, ']')) collecting = false;
            }
        } else if (collecting) {
            strncat(rbuf, s, sizeof(rbuf) - strlen(rbuf) - 1);
            if (strchr(s, ']')) collecting = false;
        }
    }
    fclose(f);
    *cnt = 0;
    char *p = rbuf;
    while (*p && *cnt < MAX_RAGGI) {
        if (*p == '[' || *p == ']') { p++; continue; }
        char *end;
        double val = strtod(p, &end);
        if (p != end) {
            ranges[(*cnt)++] = val;
            p = end;
        } else {
            p++;
        }
    }
}

void save_parsed(const char *out, double amin, double amax, double ainc,
                 double rmin, double rmax, const double ranges[], int cnt) {
    FILE *f = fopen(out, "wb");
    if (!f) return;
    fprintf(f, "angle_min=%.10f\nangle_max=%.10f\nangle_increment=%.10f\n", amin, amax, ainc);
    fprintf(f, "range_min=%.10f\nrange_max=%.10f\ncount=%d\nranges=", rmin, rmax, cnt);
    for (int i = 0; i < cnt; i++) {
        fprintf(f, "%.6f%s", ranges[i], (i < cnt-1) ? "," : "\n");
    }
    fclose(f);
}

bool load_parsed(const char *in, double *amin, double *amax, double *ainc,
        double *rmin, double *rmax, double ranges[], int *cnt) {
    FILE *f = fopen(in, "r");
    if (!f) return false;
    char line[65536];
    *cnt = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        if (sscanf(line, "angle_min=%lf", amin) == 1) continue;
        if (sscanf(line, "angle_max=%lf", amax) == 1) continue;
        if (sscanf(line, "angle_increment=%lf", ainc) == 1) continue;
        if (sscanf(line, "range_min=%lf", rmin) == 1) continue;
        if (sscanf(line, "range_max=%lf", rmax) == 1) continue;
        if (sscanf(line, "count=%d", cnt) == 1) continue;
        if (strncmp(line, "ranges=", 7) == 0) {
            char *p = line + 7;
            *cnt = 0;
            while (*p && *cnt < MAX_RAGGI) {
                ranges[(*cnt)++] = strtod(p, &p);
                if (*p == ',') p++;
            }
        }
    }
    fclose(f);
    return (*cnt > 0);
}
