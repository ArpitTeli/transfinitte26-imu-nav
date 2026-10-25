// fusion_dll.c - thin C wrapper so Python (ctypes) can drive the project's fusion filter (fusion.c, unmodified copy).
#include "fusion.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

typedef struct { FusionParams p; Fusion f; } Handle;

EXPORT void *fl_create(void) {
    Handle *h = calloc(1, sizeof *h);
    fusion_default_params(&h->p);
    return h;
}
EXPORT void fl_free(void *hp) { free(hp); }

// returns 0 if the key is unknown
EXPORT int fl_set(void *hp, const char *k, float x) {
    FusionParams *p = &((Handle *)hp)->p;
    if      (!strcmp(k,"enc_v"))     p->use_enc_v = x != 0;
    else if (!strcmp(k,"enc_w"))     p->use_enc_w = x != 0;
    else if (!strcmp(k,"mag"))       p->use_mag = x != 0;
    else if (!strcmp(k,"zupt"))      p->use_zupt = x != 0;
    else if (!strcmp(k,"r_mag"))     p->r_mag = x;
    else if (!strcmp(k,"sig_gyro"))  p->sig_gyro = x;
    else if (!strcmp(k,"sig_accel")) p->sig_accel = x;
    else if (!strcmp(k,"sig_bg"))    p->sig_bg = x;
    else if (!strcmp(k,"sig_ba"))    p->sig_ba = x;
    else if (!strcmp(k,"gate"))      p->gate = x;
    else if (!strcmp(k,"beta"))      p->beta = x;
    else if (!strcmp(k,"dip_tol"))   p->mag_dip_tol = x;
    else if (!strcmp(k,"step_tol"))  p->mag_step_tol = x;
    else if (!strcmp(k,"mag_tol"))   p->mag_norm_tol = x;
    else if (!strcmp(k,"mag_ref_s")) p->mag_ref_s = x;
    else if (!strcmp(k,"zupt_v"))    p->zupt_v = x;
    else if (!strcmp(k,"zupt_w"))    p->zupt_w = x;
    else if (!strcmp(k,"zupt_a"))    p->zupt_a = x;
    else if (!strcmp(k,"zupt_s"))    p->zupt_s = x;
    else if (!strcmp(k,"r_zupt_v"))  p->r_zupt_v = x;
    else if (!strcmp(k,"r_zupt_w"))  p->r_zupt_w = x;
    else if (!strcmp(k,"zb_min"))    p->zupt_bias_min_s = x;
    else if (!strcmp(k,"zb_mode"))   p->zupt_bias_mode = (int)x;
    else if (!strcmp(k,"recover"))   p->gate_recover_s = x;
    else return 0;
    return 1;
}

// a: m/s^2 mean over still period, g: rad/s mean (pass has_g = 0 if the phone was not still)
EXPORT void fl_init(void *hp, const float *a, const float *g, int has_g) {
    Handle *h = hp;
    fusion_init(&h->f, &h->p, a, has_g ? g : NULL);
}

// out[16]: x, y, yaw(filter), v, bg, ba, roll, pitch, yaw_madgwick(unused-ish), stationary,
//          mag_ok, mag_rej, zupt_ok, zupt_rej, mag_ref_set, 0
EXPORT void fl_step(void *hp, const float *a, const float *g, const float *m, int has_mag, float dt, float *out) {
    Handle *h = hp; Fusion *f = &h->f;
    Sample s; memcpy(s.a, a, 12); memcpy(s.g, g, 12); memcpy(s.m, m, 12);
    s.has_mag = has_mag != 0; s.vL = s.vR = 0; s.dt = dt;
    fusion_step(f, &s);
    float w = f->q[0], x = f->q[1], y = f->q[2], z = f->q[3];
    out[0] = f->x[X_PX]; out[1] = f->x[X_PY]; out[2] = f->x[X_YAW]; out[3] = f->x[X_VEL];
    out[4] = f->x[X_BG]; out[5] = f->x[X_BA];
    out[6] = atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y));
    float sp = 2*(w*y - z*x); if (sp > 1) sp = 1; if (sp < -1) sp = -1;
    out[7] = asinf(sp);
    out[8] = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z));
    out[9] = f->stationary;
    out[10] = (float)f->n_ok[S_MAG];  out[11] = (float)f->n_rej[S_MAG];
    out[12] = (float)f->n_ok[S_ZUPT]; out[13] = (float)f->n_rej[S_ZUPT];
    out[14] = f->mag_ref_set; out[15] = 0;
}
