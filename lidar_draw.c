#include "lidar.h"

static int conv_x(double x_m) {
    return (int)lround(WIN_W / 2.0 + x_m * PIX_SCALE);
}
static int conv_y(double y_m) {
    return (int)lround(WIN_H / 2.0 - y_m * PIX_SCALE);
}

static void draw_grid(SDL_Renderer *r, double rmax) {
    SDL_SetRenderDrawColor(r, 60, 60, 70, 255);
    double step = 0.5;
    for (double v = -rmax; v <= rmax; v += step) {
        SDL_RenderLine(r, (float)conv_x(v), (float)conv_y(-rmax),
                       (float)conv_x(v), (float)conv_y(rmax));
        SDL_RenderLine(r, (float)conv_x(-rmax), (float)conv_y(v),
                       (float)conv_x(rmax), (float)conv_y(v));
    }
}


static void draw_axes(SDL_Renderer *r, TTF_Font *font, double rmax) {
    SDL_SetRenderDrawColor(r, 200, 80, 80, 255);
    for (int o = -1; o <= 1; o++) {
        SDL_RenderLine(r, (float)conv_x(-rmax), (float)conv_y(0) + o,
                       (float)conv_x(rmax), (float)conv_y(0) + o);}
    SDL_SetRenderDrawColor(r, 80, 200, 80, 255);
    for (int o = -1; o <= 1; o++) {
        SDL_RenderLine(r, (float)conv_x(0) + o, (float)conv_y(-rmax),
                       (float)conv_x(0) + o, (float)conv_y(rmax));}
    
    SDL_Color txt = {180, 180, 180, 255};
    for (int v = (int)(-rmax); v <= (int)rmax; v++) {
        if (v == 0) continue;
        char lbl[8];
        snprintf(lbl, sizeof(lbl), "%d", v);
        SDL_Surface *s = TTF_RenderText_Blended(font, lbl, strlen(lbl), txt);
        if (!s) continue;
        SDL_Texture *t = SDL_CreateTextureFromSurface(r, s);
        SDL_FRect dx = {(float)conv_x((double)v) - s->w/2.0f, (float)conv_y(0) + 8, 
                        (float)s->w, (float)s->h};
        SDL_RenderTexture(r, t, NULL, &dx);
        SDL_FRect dy = {(float)conv_x(0) - s->w - 8, (float)conv_y((double)v) - s->h/2.0f,
                        (float)s->w, (float)s->h};
        SDL_RenderTexture(r, t, NULL, &dy);
        SDL_DestroyTexture(t);
        SDL_DestroySurface(s);}}
static void draw_legend(SDL_Renderer *r, TTF_Font *font, const Retta lines[], int nL,
      const Inter ints[], int nI, int closest) {
    int x = 20, y = 20, sp = 22;
    SDL_Color bg = {20, 20, 20, 220};
    SDL_Color txt = {240, 240, 240, 255};
    int h = 60 + nL * sp + (closest >= 0 ? sp : 0);
    SDL_SetRenderDrawColor(r, bg.r, bg.g, bg.b, bg.a);
    SDL_FRect panel = {10, 10, 280, (float)h};
    SDL_RenderFillRect(r, &panel);
    SDL_SetRenderDrawColor(r, 100, 100, 100, 255);
    SDL_RenderRect(r, &panel);
    SDL_Surface *s = TTF_RenderText_Blended(font, "TESPIT EDILEN DOGRULAR", 22, txt);
    SDL_Texture *t = SDL_CreateTextureFromSurface(r, s);
    SDL_FRect d = {(float)x, (float)y, (float)s->w, (float)s->h};
    SDL_RenderTexture(r, t, NULL, &d);
    SDL_DestroyTexture(t);
    SDL_DestroySurface(s);
    y += sp;
    for (int i = 0; i < nL; i++) {
        double len = hypot(lines[i].p_end.x - lines[i].p_start.x,
                          lines[i].p_end.y - lines[i].p_start.y);
        char buf[128];
        snprintf(buf, sizeof(buf), "D%d: %.2f m (%d pt)", i+1, len,
                abs(lines[i].idx_end - lines[i].idx_start + 1));
        s = TTF_RenderText_Blended(font, buf, strlen(buf), txt);
        t = SDL_CreateTextureFromSurface(r, s);
        d = (SDL_FRect){(float)x, (float)y, (float)s->w, (float)s->h};
        SDL_RenderTexture(r, t, NULL, &d);
        SDL_DestroyTexture(t);
        SDL_DestroySurface(s);
        y += sp;}
    if (closest >= 0 && closest < nI) {
        char buf[128];
        snprintf(buf, sizeof(buf), "En yakin: %.2f m", ints[closest].dist);
        s = TTF_RenderText_Blended(font, buf, strlen(buf), txt);
        t = SDL_CreateTextureFromSurface(r, s);
        d = (SDL_FRect){(float)x, (float)y, (float)s->w, (float)s->h};
        SDL_RenderTexture(r, t, NULL, &d);
        SDL_DestroyTexture(t);
        SDL_DestroySurface(s);}}

bool draw_viz(const Punto pts[], int nP, const Retta lines[], int nL,
              const Inter ints[], int nI, int closest, double rmax) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return false;
    if (TTF_Init() < 0) {
        SDL_Quit();
        return false;
    }
    SDL_Window *win = SDL_CreateWindow("LIDAR", WIN_W, WIN_H, 0);
    SDL_Renderer *rend = SDL_CreateRenderer(win, NULL);
    TTF_Font *font = TTF_OpenFont("C:\\Windows\\Fonts\\arial.ttf", 14);
    if (!win || !rend || !font) {
        if (font) TTF_CloseFont(font);
        if (rend) SDL_DestroyRenderer(rend);
        if (win) SDL_DestroyWindow(win);
        TTF_Quit();
        SDL_Quit();
        return false;
    }
    SDL_SetRenderDrawColor(rend, 25, 25, 35, 255);
    SDL_RenderClear(rend);
    draw_grid(rend, rmax);
    draw_axes(rend, font, rmax);
    SDL_SetRenderDrawColor(rend, 220, 80, 80, 255);
    for (int a = 0; a < 360; a++) {
        double rad = a * M_PI / 180.0;
        int px = conv_x(0.08 * cos(rad));
        int py = conv_y(0.08 * sin(rad));
        SDL_RenderPoint(rend, (float)px, (float)py);
    }
    SDL_SetRenderDrawColor(rend, 80, 160, 255, 255);
    for (int i = 0; i < nP; i++) {
        SDL_RenderPoint(rend, (float)conv_x(pts[i].x), (float)conv_y(pts[i].y));
    }
    SDL_SetRenderDrawColor(rend, 80, 220, 120, 255);
    for (int i = 0; i < nL; i++) {
        double dx = lines[i].p_end.x - lines[i].p_start.x;
        double dy = lines[i].p_end.y - lines[i].p_start.y;
        double len = hypot(dx, dy);
        if (len < 0.01) continue;
        dx /= len; dy /= len;
        double ext = len * 0.3;
        double x1 = lines[i].p_start.x - dx * ext;
        double y1 = lines[i].p_start.y - dy * ext;
        double x2 = lines[i].p_end.x + dx * ext;
        double y2 = lines[i].p_end.y + dy * ext;
        SDL_RenderLine(rend, (float)conv_x(x1), (float)conv_y(y1),
                      (float)conv_x(x2), (float)conv_y(y2));
        
        double mx = (lines[i].p_start.x + lines[i].p_end.x) / 2.0;
        double my = (lines[i].p_start.y + lines[i].p_end.y) / 2.0;
        char lbl[16];
        snprintf(lbl, sizeof(lbl), "d%d", i + 1);
        SDL_Color lc = {100, 255, 120, 255};
        SDL_Surface *s = TTF_RenderText_Blended(font, lbl, strlen(lbl), lc);
        if (s) {
            SDL_Texture *t = SDL_CreateTextureFromSurface(rend, s);
            SDL_FRect d = {(float)conv_x(mx) + 8, (float)conv_y(my) - 8, (float)s->w, (float)s->h};
            SDL_RenderTexture(rend, t, NULL, &d);
            SDL_DestroyTexture(t);
            SDL_DestroySurface(s);}}
    for (int i = 0; i < nI; i++) {
        SDL_SetRenderDrawColor(rend, 255, 220, 60, 255);
        double cs = 0.1;
        SDL_RenderLine(rend, (float)conv_x(ints[i].x - cs), (float)conv_y(ints[i].y),
                      (float)conv_x(ints[i].x + cs), (float)conv_y(ints[i].y));
        SDL_RenderLine(rend, (float)conv_x(ints[i].x), (float)conv_y(ints[i].y - cs),
                      (float)conv_x(ints[i].x), (float)conv_y(ints[i].y + cs));
        
        for (int a = 0; a < 360; a += 3) {
            double rad = a * M_PI / 180.0;
            int px = conv_x(ints[i].x + 0.06 * cos(rad));
            int py = conv_y(ints[i].y + 0.06 * sin(rad));
            SDL_RenderPoint(rend, (float)px, (float)py);
     }
        char albl[64];
        snprintf(albl, sizeof(albl), "%.0f° (d%d^d%d)", ints[i].ang, 
                ints[i].idx1+1, ints[i].idx2+1);
        SDL_Color ac = {255, 255, 100, 255};
        SDL_Surface *s = TTF_RenderText_Blended(font, albl, strlen(albl), ac);
        if (s) { SDL_Texture *t = SDL_CreateTextureFromSurface(rend, s); 
            double dx = ints[i].x;
            double dy = ints[i].y;
            double len = hypot(dx, dy);
            if (len > 0.01) { dx /= len; dy /= len;
            } else { dx = 1.0; dy = 0.0;  }
            float loff = 60.0f; 


            SDL_SetRenderDrawColor(rend, 40, 40, 40, 200);
            SDL_FRect bg = {(float)conv_x(ints[i].x) + dx * loff - 4,
                           (float)conv_y(ints[i].y) - dy * loff - 4,
                           (float)s->w + 8, (float)s->h + 8};
            SDL_RenderFillRect(rend, &bg);
            
            SDL_FRect d = {(float)conv_x(ints[i].x) + dx * loff,
                          (float)conv_y(ints[i].y) - dy * loff,
                          (float)s->w, (float)s->h};
            SDL_RenderTexture(rend, t, NULL, &d);
            SDL_DestroyTexture(t);
            SDL_DestroySurface(s);}}
    if (closest >= 0 && closest < nI) {
        SDL_SetRenderDrawColor(rend, 255, 120, 120, 255);
        double x1 = 0, y1 = 0;
        double x2 = ints[closest].x;
        double y2 = ints[closest].y;
        double dist = hypot(x2 - x1, y2 - y1);
        int steps = (int)(dist * PIX_SCALE / 6.0);
        if (steps < 1) steps = 1;
        double dx = x2 / steps;
        double dy = y2 / steps;
        for (int i = 0; i < steps; i++) {
            if (i % 2 == 0) {
                double sx = dx * i;
                double sy = dy * i;
                double ex = dx * (i + 1);
                double ey = dy * (i + 1);
                SDL_RenderLine(rend, (float)conv_x(sx), (float)conv_y(sy),
                              (float)conv_x(ex), (float)conv_y(ey));}}
        SDL_Color lc = {255, 240, 100, 255};
        char msg[64];
        snprintf(msg, sizeof(msg), "d = %.2f m", ints[closest].dist);
        SDL_Surface *s = TTF_RenderText_Blended(font, msg, strlen(msg), lc);
        SDL_Texture *t = SDL_CreateTextureFromSurface(rend, s);
        SDL_FRect d = {(float)conv_x(ints[closest].x + 0.15),
                      (float)conv_y(ints[closest].y + 0.15),
                      (float)s->w, (float)s->h};
        SDL_RenderTexture(rend, t, NULL, &d);
        SDL_DestroyTexture(t);
        SDL_DestroySurface(s);}
    draw_legend(rend, font, lines, nL, ints, nI, closest);
    SDL_RenderPresent(rend);
    bool run = true, cont = true;
    while (run) {
          SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) {
                int r = MessageBoxA(NULL, "Yeni dosya acmak ister misin?", "Devam", MB_YESNO | MB_ICONQUESTION);
                run = false;
                cont = (r == IDYES);}}
        SDL_DelayNS(16000000LL);}
    TTF_CloseFont(font);
    SDL_DestroyRenderer(rend);
    SDL_DestroyWindow(win);
    TTF_Quit();
    SDL_Quit();
    return cont;
   }
