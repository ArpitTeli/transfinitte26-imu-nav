// replay.c - run the fusion core on a CSV log and write the estimated trajectory.
// Input CSV columns (header line allowed):
//   t[s], ax,ay,az [m/s^2], gx,gy,gz [rad/s], mx,my,mz, vL,vR [m/s]
// Usage: ./replay in.csv out.csv [key=value ...]
//   still=N        average first N rows as the startup still period (0 = don't)
//   enc_v=0/1 enc_w=0/1 mag=0/1 zupt=0/1      sensor switches
//   r_mag=.. r_enc_v=.. r_enc_w=.. sig_gyro=.. sig_accel=.. gate=.. track=..
//   moff=x,y,z     magnetometer hard-iron offset
//   zupt_v= zupt_w= zupt_a= zupt_s= r_zupt_v= r_zupt_w= recover= vscale= mag_ref_s= mag_tol= dip_tol= step_tol= beta= sig_bg= sig_ba=
// An unknown key is an error (exit 2): a typo or a renamed option must not silently fall back to the default.
#include "fusion.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct { double t; float a[3], g[3], m[3], vL, vR; } Row;

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: replay in.csv out.csv [key=value...]\n"); return 1; }
    FusionParams p; fusion_default_params(&p);
    int still = 0; float vscale = 1.0f;
    for (int i = 3; i < argc; i++) {
        char *eq = strchr(argv[i], '=');
        if (!eq) { fprintf(stderr, "replay: expected key=value, got '%s'\n", argv[i]); return 2; }
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
        else if (!strcmp(k,"dip_tol"))   p.mag_dip_tol = x;
        else if (!strcmp(k,"step_tol"))  p.mag_step_tol = x;
        else if (!strcmp(k,"vscale"))    vscale = x;
        else if (!strcmp(k,"pos_consider")) p.pos_consider = (uint8_t)x;   // bitmask: 1 enc_v, 2 enc_w, 4 mag, 8 zupt
        else if (!strcmp(k,"zb_mode"))   p.zupt_bias_mode = (int)x;
        else if (!strcmp(k,"zb_min"))    p.zupt_bias_min_s = x;
        else if (!strcmp(k,"zb_max"))    p.zupt_bias_max_s = x;
        else if (!strcmp(k,"zb_floor"))  p.zupt_bias_floor = x;
        else if (!strcmp(k,"zb_lag"))    p.zupt_bias_lag_s = x;
        else if (!strcmp(k,"recover"))   p.gate_recover_s = x;
        else if (!strcmp(k,"zupt_v"))    p.zupt_v = x;
        else if (!strcmp(k,"zupt_w"))    p.zupt_w = x;
        else if (!strcmp(k,"zupt_a"))    p.zupt_a = x;
        else if (!strcmp(k,"zupt_s"))    p.zupt_s = x;
        else if (!strcmp(k,"r_zupt_v"))  p.r_zupt_v = x;
        else if (!strcmp(k,"r_zupt_w"))  p.r_zupt_w = x;
        else if (!strcmp(k,"mag_ref_s")) p.mag_ref_s = x;
        else if (!strcmp(k,"mag_tol"))   p.mag_norm_tol = x;
        else if (!strcmp(k,"track"))     p.track_width = x;
        else if (!strcmp(k,"moff")) {
            if (sscanf(v, "%f,%f,%f", &p.mag_off[0], &p.mag_off[1], &p.mag_off[2]) != 3) {
                fprintf(stderr, "replay: moff needs three comma-separated numbers\n"); return 2;
            }
        }
        else { fprintf(stderr, "replay: unknown option '%s'\n", k); return 2; }
    }
    FILE *fi = fopen(argv[1], "r"); if (!fi) { perror(argv[1]); return 1; }
    size_t cap = 1 << 16, n = 0; char line[512];
    Row *rows = malloc(sizeof(Row) * cap);
    if (!rows) { fprintf(stderr, "replay: out of memory\n"); fclose(fi); return 1; }
    while (fgets(line, sizeof line, fi)) {          // no row limit (a 400 Hz phone log passes 2M rows after 83 min)
        Row r; double t;
        if (sscanf(line, "%lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t, &r.a[0], &r.a[1], &r.a[2],
                   &r.g[0], &r.g[1], &r.g[2], &r.m[0], &r.m[1], &r.m[2], &r.vL, &r.vR) != 12) continue;
        if (n == cap) {
            Row *nr = realloc(rows, sizeof(Row) * cap * 2);
            if (!nr) { fprintf(stderr, "replay: out of memory at %zu rows\n", n); free(rows); fclose(fi); return 1; }
            rows = nr; cap *= 2;
        }
        r.t = t; rows[n++] = r;
    }
    fclose(fi);
    if (n < 10) { fprintf(stderr, "too few rows\n"); free(rows); return 1; }

    for (size_t i = 0; i < n; i++) { rows[i].vL *= vscale; rows[i].vR *= vscale; }
    // startup average: the given still period, or the first 20 rows for the tilt only (never past the end)
    size_t ns = still > 0 && (size_t)still < n ? (size_t)still : 0;
    size_t na = ns ? ns : (n < 20 ? n : 20);
    float a0[3] = {0}, g0[3] = {0};
    for (size_t i = 0; i < na; i++) for (int k = 0; k < 3; k++) { a0[k] += rows[i].a[k]; g0[k] += rows[i].g[k]; }
    for (int k = 0; k < 3; k++) { a0[k] /= na; g0[k] /= na; }

    FILE *fo = fopen(argv[2], "w");
    if (!fo) { perror(argv[2]); free(rows); return 1; }
    Fusion f; fusion_init(&f, &p, a0, ns ? g0 : NULL);
    fprintf(fo, "t,x,y,yaw,v,bg,ba,stationary\n");
    for (size_t i = 1; i < n; i++) {
        Sample s; memcpy(s.a, rows[i].a, 12); memcpy(s.g, rows[i].g, 12); memcpy(s.m, rows[i].m, 12);
        s.has_mag = isfinite(s.m[0]); s.vL = rows[i].vL; s.vR = rows[i].vR; s.dt = (float)(rows[i].t - rows[i-1].t);
        fusion_step(&f, &s);
        fprintf(fo, "%.4f,%.5f,%.5f,%.5f,%.5f,%.6f,%.5f,%d\n", rows[i].t, f.x[X_PX], f.x[X_PY], f.x[X_YAW], f.x[X_VEL], f.x[X_BG], f.x[X_BA], f.stationary);
    }
    fclose(fo);
    fprintf(stderr, "accepted/rejected  enc_v %u/%u  enc_w %u/%u  mag %u/%u  zupt %u/%u\n",
            f.n_ok[S_ENC_V], f.n_rej[S_ENC_V], f.n_ok[S_ENC_W], f.n_rej[S_ENC_W],
            f.n_ok[S_MAG], f.n_rej[S_MAG], f.n_ok[S_ZUPT], f.n_rej[S_ZUPT]);
    free(rows); return 0;
}
