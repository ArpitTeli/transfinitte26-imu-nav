// replay.c - run the fusion core on a CSV log and write the estimated trajectory.
// Input CSV columns (header line allowed):
//   t[s], ax,ay,az [m/s^2], gx,gy,gz [rad/s], mx,my,mz, vL,vR [m/s]
// Usage: ./replay in.csv out.csv [key=value ...]
//   still=N        average first N rows as the startup still period (0 = don't)
//   enc_v=0/1 enc_w=0/1 mag=0/1 zupt=0/1      sensor switches
//   r_mag=.. r_enc_v=.. r_enc_w=.. sig_gyro=.. sig_accel=.. gate=.. track=..
//   moff=x,y,z     magnetometer hard-iron offset
#include "fusion.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAXROWS 2000000
typedef struct { double t; float a[3], g[3], m[3], vL, vR; } Row;

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: replay in.csv out.csv [key=value...]\n"); return 1; }
    FusionParams p; fusion_default_params(&p);
    int still = 0;
    for (int i = 3; i < argc; i++) {
        char *eq = strchr(argv[i], '='); if (!eq) continue;
        *eq = 0; const char *k = argv[i]; const char *v = eq + 1; float x = (float)atof(v);
        if      (!strcmp(k,"still"))     still = (int)x;
        else if (!strcmp(k,"enc_v"))     p.use_enc_v = x != 0;
        else if (!strcmp(k,"enc_w"))     p.use_enc_w = x != 0;
        else if (!strcmp(k,"mag"))       p.use_mag = x != 0;
        else if (!strcmp(k,"zupt"))      p.use_zupt = x != 0;
        else if (!strcmp(k,"r_mag"))     p.r_mag = x;
        else if (!strcmp(k,"r_enc_v"))   p.r_enc_v = x;
        else if (!strcmp(k,"r_enc_w"))   p.r_enc_w = x;
        else if (!strcmp(k,"sig_gyro"))  p.sig_gyro = x;
        else if (!strcmp(k,"sig_accel")) p.sig_accel = x;
        else if (!strcmp(k,"sig_bg"))    p.sig_bg = x;
        else if (!strcmp(k,"sig_ba"))    p.sig_ba = x;
        else if (!strcmp(k,"gate"))      p.gate = x;
        else if (!strcmp(k,"beta"))      p.beta = x;
        else if (!strcmp(k,"track"))     p.track_width = x;
        else if (!strcmp(k,"moff"))      sscanf(v, "%f,%f,%f", &p.mag_off[0], &p.mag_off[1], &p.mag_off[2]);
    }
    FILE *fi = fopen(argv[1], "r"); if (!fi) { perror("in"); return 1; }
    Row *rows = malloc(sizeof(Row) * MAXROWS); int n = 0; char line[512];
    while (fgets(line, sizeof line, fi) && n < MAXROWS) {
        Row r; double t;
        if (sscanf(line, "%lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t, &r.a[0], &r.a[1], &r.a[2],
                   &r.g[0], &r.g[1], &r.g[2], &r.m[0], &r.m[1], &r.m[2], &r.vL, &r.vR) == 12) { r.t = t; rows[n++] = r; }
    }
    fclose(fi);
    if (n < 10) { fprintf(stderr, "too few rows\n"); return 1; }

    float a0[3] = {0}, g0[3] = {0};
    int ns = still > 0 && still < n ? still : 0;
    for (int i = 0; i < (ns ? ns : 20); i++) for (int k = 0; k < 3; k++) { a0[k] += rows[i].a[k]; g0[k] += rows[i].g[k]; }
    for (int k = 0; k < 3; k++) { a0[k] /= (ns ? ns : 20); g0[k] /= (ns ? ns : 20); }

    Fusion f; fusion_init(&f, &p, a0, ns ? g0 : NULL);
    FILE *fo = fopen(argv[2], "w"); fprintf(fo, "t,x,y,yaw,v,bg,ba,stationary\n");
    for (int i = 1; i < n; i++) {
        Sample s; memcpy(s.a, rows[i].a, 12); memcpy(s.g, rows[i].g, 12); memcpy(s.m, rows[i].m, 12);
        s.has_mag = isfinite(s.m[0]); s.vL = rows[i].vL; s.vR = rows[i].vR; s.dt = (float)(rows[i].t - rows[i-1].t);
        fusion_step(&f, &s);
        fprintf(fo, "%.4f,%.5f,%.5f,%.5f,%.5f,%.6f,%.5f,%d\n", rows[i].t, f.x[PX], f.x[PY], f.x[YAW], f.x[VEL], f.x[BG], f.x[BA], f.stationary);
    }
    fclose(fo);
    fprintf(stderr, "accepted/rejected  enc_v %u/%u  enc_w %u/%u  mag %u/%u  zupt %u/%u\n",
            f.n_ok[S_ENC_V], f.n_rej[S_ENC_V], f.n_ok[S_ENC_W], f.n_rej[S_ENC_W],
            f.n_ok[S_MAG], f.n_rej[S_MAG], f.n_ok[S_ZUPT], f.n_rej[S_ZUPT]);
    free(rows); return 0;
}
