#include "lidar.h"

int main(void) {
    bool cont = true;
    while (cont) {
        int num;
        char file[32], pfile[32], url[256];
        double amin = 0, amax = 0, ainc = 0, rmin = 0, rmax = 0;
        double ranges[MAX_RAGGI];
        int nranges = 0;
        double fr[MAX_RAGGI], fa[MAX_RAGGI];
        Punto fp[MAX_RAGGI];
        int nfilt = 0;
        Retta lines[MAX_RETTE];
        int nlines = 0;
        Inter ints[MAX_RETTE * MAX_RETTE];
        int nints = 0, closest = -1;
        
        printf("\n------------------------------------------------------------\n");
        printf("Istedigni LIDAR dosya sec (1-5) yada 0 (cikmak icin):");
        if (scanf("%d", &num) != 1) {
            while (getchar() != '\n');
            continue;}
        if (num == 0) {
            if (MessageBoxA(NULL, "Cikis?", "Onay", MB_YESNO | MB_ICONQUESTION) == IDYES) {
                break;
            }
            continue;
        }
        if (num < 1 || num > 5) {
            printf("Hatali sectin!\n");
            continue;
        }// dont forget to add the uni links 
        static const char *urls[] = {
            "http://abilgisayar.kocaeli.edu.tr/lidar1.toml",
            "http://abilgisayar.kocaeli.edu.tr/lidar2.toml",
            "http://abilgisayar.kocaeli.edu.tr/lidar3.toml",
            "http://abilgisayar.kocaeli.edu.tr/lidar4.toml",
            "http://abilgisayar.kocaeli.edu.tr/lidar5.toml"

        };
 
        snprintf(file, sizeof(file), "lidar%d.toml", num);
        snprintf(pfile, sizeof(pfile), "lidar%dp.toml", num);
        snprintf(url, sizeof(url), "%s", urls[num - 1]);
        

        if (load_parsed(pfile, &amin, &amax, &ainc, &rmin, &rmax, ranges, &nranges)) {
            printf("OK Onceden islenmis dosya yuklendi.\n");
        } else {
            FILE *f = fopen(file, "r");
            if (!f) {
                printf("Indiriliyor...\n");
                download(url, file);
            } else {
                fclose(f);
            }
            parse_toml(file, &amin, &amax, &ainc, &rmin, &rmax, ranges, &nranges);
            save_parsed(pfile, amin, amax, ainc, rmin, rmax, ranges, nranges);
        }
        
        nfilt = filter_pts(ranges, nranges, rmin, rmax, amin, ainc, amax, fr, fa, fp);
        printf("%d nokta filtrelendi.\n", nfilt);
        
        printf("Dogrulari tespit ediliyor...\n");
        nlines = detect_lines(fp, nfilt, lines);
        printf("%d dogru bulundu.\n", nlines);
        
        nints = calc_intersect(lines, nlines, rmax, ints, &closest);
        
        if (nints > 0) {
            printf("\nKesisimler (60+ derece):\n");
            int shown = 0;
            for (int i = 0; i < nints; i++) {
                if (ints[i].ang >= 60.0 && ints[i].ang <= 120.0) {
                    printf("  %.0f derece (d%d^d%d) - %.2fm\n", ints[i].ang, 
                           ints[i].idx1+1, ints[i].idx2+1, ints[i].dist);
                    shown++;
                }
            }
            if (shown > 0 && closest >= 0) {
                printf("En yakin: %.2f m\n\n", ints[closest].dist);
            }
        }
        
        cont = draw_viz(fp, nfilt, lines, nlines, ints, nints, closest, rmax);
    }
    
    return 0;
}
