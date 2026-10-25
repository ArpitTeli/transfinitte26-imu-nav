#include "fusion.h"
#include <math.h>
#include <string.h>

#define G0 9.81f
#define PI_F 3.14159265f

static float wrap(float a) {
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

void fusion_default_params(FusionParams *p) {
    memset(p, 0, sizeof(*p));
    p->sig_gyro = 0.01f;   p->sig_accel = 0.2f;
    p->sig_bg   = 1e-4f;   p->sig_ba    = 1e-3f;
    p->r_enc_v  = 0.03f;   p->r_enc_w   = 0.05f;   p->r_mag = 0.15f;
    p->r_zupt_v = 0.005f;  p->r_zupt_w  = 0.01f;   p->r_zupt_a = 0.1f;
    p->beta = 0.02f;
    p->gate = 3.0f;
    p->track_width = 0.15f;
    p->zupt_v = 0.004f;  p->zupt_w = 0.015f;  p->zupt_a = 0.15f;  p->zupt_samples = 100;
    p->use_enc_v = p->use_enc_w = p->use_mag = p->use_zupt = true;
    for (int i = 0; i < 3; i++) p->mag_scale[i] = 1.0f;
    p->mag_norm_ref = 0.0f;  p->mag_norm_tol = 0.15f;  p->mag_ref_samples = 400;
}

void fusion_init(Fusion *f, const FusionParams *p, const float a[3], const float g[3]) {
    memset(f, 0, sizeof(*f));
    f->p = *p;
    f->beta = p->beta;
    // initial roll/pitch from gravity, yaw = 0
    float roll  = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));
    float cr = cosf(roll/2), sr = sinf(roll/2), cp = cosf(pitch/2), sp = sinf(pitch/2);
    f->q[0] = cp*cr;  f->q[1] = cp*sr;  f->q[2] = sp*cr;  f->q[3] = -sp*sr;
    if (g) for (int i = 0; i < 3; i++) f->gbias0[i] = g[i];
    f->P[PX][PX] = f->P[PY][PY] = 1e-4f;
    f->P[YAW][YAW] = 1e-4f;
    f->P[VEL][VEL] = 0.01f;
    f->P[BG][BG]   = 1e-3f;
    f->P[BA][BA]   = 0.01f;
    f->P[EPS][EPS] = 1e-4f;
}

// ---------------- Madgwick (6-axis: gyro + accel). Only tilt is trusted, yaw is not used. ------
static void madgwick(Fusion *f, const float g[3], const float a_in[3], float dt) {
    float q0=f->q[0], q1=f->q[1], q2=f->q[2], q3=f->q[3];
    float n = sqrtf(a_in[0]*a_in[0] + a_in[1]*a_in[1] + a_in[2]*a_in[2]);
    float gx=g[0], gy=g[1], gz=g[2];
    float qd0 = 0.5f*(-q1*gx - q2*gy - q3*gz);
    float qd1 = 0.5f*( q0*gx + q2*gz - q3*gy);
    float qd2 = 0.5f*( q0*gy - q1*gz + q3*gx);
    float qd3 = 0.5f*( q0*gz + q1*gy - q2*gx);
    if (n > 1e-3f) {
        float ax=a_in[0]/n, ay=a_in[1]/n, az=a_in[2]/n;
        float f1 = 2.0f*(q1*q3 - q0*q2) - ax;
        float f2 = 2.0f*(q0*q1 + q2*q3) - ay;
        float f3 = q0*q0 - q1*q1 - q2*q2 + q3*q3 - az;
        float s0 = -2.0f*q2*f1 + 2.0f*q1*f2 + 2.0f*q0*f3;
        float s1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 2.0f*q1*f3;
        float s2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 2.0f*q2*f3;
        float s3 =  2.0f*q1*f1 + 2.0f*q2*f2 + 2.0f*q3*f3;
        float sn = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (sn > 1e-9f) {
            qd0 -= f->beta*s0/sn; qd1 -= f->beta*s1/sn;
            qd2 -= f->beta*s2/sn; qd3 -= f->beta*s3/sn;
        }
    }
    q0 += qd0*dt; q1 += qd1*dt; q2 += qd2*dt; q3 += qd3*dt;   // <- integrate (was missing in the old file)
    float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    f->q[0]=q0/qn; f->q[1]=q1/qn; f->q[2]=q2/qn; f->q[3]=q3/qn;
}

// ---------------- Kalman helpers ----------------
static void predict(Fusion *f, float wz, float a_fwd, float dt) {
    FusionParams *p = &f->p;
    float psi = f->x[YAW], v = f->x[VEL];
    float c = cosf(psi), s = sinf(psi);

    float F[NS][NS] = {{0}};
    for (int i = 0; i < NS; i++) F[i][i] = 1.0f;
    F[PX][YAW] = -v*s*dt;  F[PX][VEL] = c*dt;
    F[PY][YAW] =  v*c*dt;  F[PY][VEL] = s*dt;
    F[YAW][BG] = -dt;      F[VEL][BA] = -dt;

    f->x[PX]  += v*c*dt;
    f->x[PY]  += v*s*dt;
    f->x[YAW]  = wrap(psi + (wz - f->x[BG])*dt);
    f->x[VEL] += (a_fwd - f->x[BA])*dt;

    float T[NS][NS] = {{0}}, Pn[NS][NS];
    for (int i = 0; i < NS; i++) for (int j = 0; j < NS; j++)
        for (int k = 0; k < NS; k++) T[i][j] += F[i][k]*f->P[k][j];
    for (int i = 0; i < NS; i++) for (int j = 0; j < NS; j++) {
        float sum = 0;
        for (int k = 0; k < NS; k++) sum += T[i][k]*F[j][k];
        Pn[i][j] = sum;
    }
    Pn[PX][PX] += 1e-8f*dt;  Pn[PY][PY] += 1e-8f*dt;
    Pn[YAW][YAW] += p->sig_gyro*p->sig_gyro*dt*dt;
    Pn[VEL][VEL] += p->sig_accel*p->sig_accel*dt*dt;
    Pn[BG][BG]   += p->sig_bg*p->sig_bg*dt;
    Pn[BA][BA]   += p->sig_ba*p->sig_ba*dt;
    Pn[EPS][EPS] += 1e-10f*dt;
    memcpy(f->P, Pn, sizeof(Pn));
}

// Scalar measurement update. y = z - h(x), H = dh/dx. Returns false if rejected by the gate.
static bool update(Fusion *f, int sensor, float y, const float H[NS], float R) {
    float PH[NS], S = R;
    for (int i = 0; i < NS; i++) {
        PH[i] = 0;
        for (int j = 0; j < NS; j++) PH[i] += f->P[i][j]*H[j];
        S += H[i]*PH[i];
    }
    f->last_innov[sensor] = y;
    if (S <= 0 || y*y > f->p.gate*f->p.gate*S) { f->n_rej[sensor]++; return false; }
    for (int i = 0; i < NS; i++) f->x[i] += PH[i]/S*y;
    float K[NS];
    for (int i = 0; i < NS; i++) K[i] = PH[i]/S;
    for (int i = 0; i < NS; i++) for (int j = 0; j < NS; j++) f->P[i][j] -= K[i]*PH[j];
    for (int i = 0; i < NS; i++) {                       // keep symmetric / positive
        for (int j = i+1; j < NS; j++) { float m = 0.5f*(f->P[i][j] + f->P[j][i]); f->P[i][j] = f->P[j][i] = m; }
        if (f->P[i][i] < 1e-12f) f->P[i][i] = 1e-12f;
    }
    f->x[YAW] = wrap(f->x[YAW]);
    f->n_ok[sensor]++;
    return true;
}

void fusion_step(Fusion *f, const Sample *s) {
    FusionParams *p = &f->p;
    float dt = s->dt;
    if (dt <= 0 || dt > 0.5f) return;

    // 1. tilt from Madgwick (gyro with startup bias removed)
    float g[3] = { s->g[0]-f->gbias0[0], s->g[1]-f->gbias0[1], s->g[2]-f->gbias0[2] };
    madgwick(f, g, s->a, dt);
    float q0=f->q[0], q1=f->q[1], q2=f->q[2], q3=f->q[3];
    // "up" direction expressed in the sensor frame
    float u[3] = { 2.0f*(q1*q3 - q0*q2), 2.0f*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3 };

    // 2. tilt-corrected inputs
    float wz    = g[0]*u[0] + g[1]*u[1] + g[2]*u[2];      // yaw rate about the vertical
    float a_fwd = s->a[0] - G0*u[0];                      // forward linear accel, gravity removed

    // 3. predict
    predict(f, wz, a_fwd, dt);

    // 4. ZUPT (stopped = wheels still AND gyro still, for NS samples)
    // detector works on ~0.25 s low-passed signals: encoder counts are coarse at 200 Hz, averaging resolves slow rotation
    float k = dt/0.25f;
    f->vLf += k*(s->vL - f->vLf);  f->vRf += k*(s->vR - f->vRf);
    f->wzf += k*((wz - f->x[BG]) - f->wzf);  f->aff += k*((a_fwd - f->x[BA]) - f->aff);
    bool still = fabsf(f->vLf) < p->zupt_v && fabsf(f->vRf) < p->zupt_v &&
                 fabsf(f->wzf) < p->zupt_w && fabsf(f->aff) < p->zupt_a;
    f->still_count = still ? f->still_count + 1 : 0;
    f->stationary = f->still_count >= p->zupt_samples;
    if (p->use_zupt && f->stationary) {
        float H[NS] = {0};
        H[VEL] = 1;  update(f, S_ZUPT, 0.0f - f->x[VEL], H, p->r_zupt_v*p->r_zupt_v);
        memset(H, 0, sizeof(H)); H[BG] = -1;
        update(f, S_ZUPT, 0.0f - (wz - f->x[BG]), H, p->r_zupt_w*p->r_zupt_w);
        memset(H, 0, sizeof(H)); H[BA] = -1;
        update(f, S_ZUPT, 0.0f - (a_fwd - f->x[BA]), H, p->r_zupt_a*p->r_zupt_a);
    }

    // 5. wheel forward speed  (checks linear motion)
    if (p->use_enc_v) {
        float H[NS] = {0}; H[VEL] = 1;
        update(f, S_ENC_V, 0.5f*(s->vL + s->vR) - f->x[VEL], H, p->r_enc_v*p->r_enc_v);
    }
    // 6. wheel yaw rate (vR-vL)/b. Model: w_enc = (wz - bg) + eps*v/b, where eps = wheel-diameter mismatch.
    //    Checks the gyro, learns gyro bias (when still) and eps (when moving). Gated, so slip is rejected.
    if (p->use_enc_w) {
        float b = p->track_width, H[NS] = {0};
        H[BG] = -1;  H[EPS] = f->x[VEL]/b;  H[VEL] = f->x[EPS]/b;
        float w_enc = (s->vR - s->vL)/b;
        update(f, S_ENC_W, w_enc - ((wz - f->x[BG]) + f->x[EPS]*f->x[VEL]/b), H, p->r_enc_w*p->r_enc_w);
    }
    // 7. magnetometer heading (absolute, tilt-compensated), skipped when the field looks disturbed
    if (p->use_mag && s->has_mag) {
        float m[3];
        for (int i = 0; i < 3; i++) m[i] = (s->m[i] - p->mag_off[i])*p->mag_scale[i];
        float mn = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
        if (p->mag_norm_ref <= 0 && mn > 0) f->p.mag_norm_ref = p->mag_norm_ref = mn;
        bool field_ok = mn > 0 && fabsf(mn - p->mag_norm_ref) < p->mag_norm_tol*p->mag_norm_ref;
        if (!field_ok) f->n_rej[S_MAG]++;
        else {
            float md = m[0]*u[0] + m[1]*u[1] + m[2]*u[2];
            float h[3] = { m[0]-md*u[0], m[1]-md*u[1], m[2]-md*u[2] };
            float fw[3] = { 1.0f - u[0]*u[0], -u[0]*u[1], -u[0]*u[2] };       // body x, horizontal part
            float lf[3] = { u[1]*fw[2]-u[2]*fw[1], u[2]*fw[0]-u[0]*fw[2], u[0]*fw[1]-u[1]*fw[0] };  // u x fw = left
            float ang = atan2f(h[0]*lf[0]+h[1]*lf[1]+h[2]*lf[2], h[0]*fw[0]+h[1]*fw[1]+h[2]*fw[2]);
            if (!f->mag_ref_set) {                       // average (ang + yaw) over the first samples
                float r = ang + f->x[YAW];
                f->mag_sin += sinf(r); f->mag_cos += cosf(r);
                if (++f->mag_n >= p->mag_ref_samples) { f->mag_ref = atan2f(f->mag_sin, f->mag_cos); f->mag_ref_set = true; }
                return;
            }
            float psi_m = f->mag_ref - ang;
            float H[NS] = {0}; H[YAW] = 1;
            update(f, S_MAG, wrap(psi_m - f->x[YAW]), H, p->r_mag*p->r_mag);
        }
    }
}
