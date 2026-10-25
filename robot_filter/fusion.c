#include "fusion.h"
#include <math.h>
#include <string.h>

#define G0 9.81f
#define PI_F 3.14159265f

// angle to [-pi, pi). Bounded time: the old subtract-in-a-loop version never returned for +-inf
// (a diverged filter would hang the whole firmware task).
static float wrap(float a) {
    if (!isfinite(a)) return a;
    if (a >= -PI_F && a < PI_F) return a;
    a = fmodf(a + PI_F, 2.0f * PI_F);
    if (a < 0) a += 2.0f * PI_F;
    return a - PI_F;
}

void fusion_default_params(FusionParams *p) {
    memset(p, 0, sizeof(*p));
    p->sig_gyro = 0.01f;   p->sig_accel = 0.2f;
    p->sig_bg   = 1e-4f;   p->sig_ba    = 1e-3f;
    p->r_enc_v  = 0.03f;   p->r_enc_w   = 0.05f;   p->r_mag = 0.15f;
    p->r_zupt_v = 0.005f;  p->r_zupt_w  = 0.01f;   p->r_zupt_a = 0.1f;
    p->beta = 0.02f;
    p->gate = 3.0f;        p->gate_recover_s = 3.0f;
    p->pos_consider = 1u << S_ZUPT;
    p->track_width = 0.15f;
    p->zupt_v = 0.004f;  p->zupt_w = 0.015f;  p->zupt_a = 0.15f;  p->zupt_s = 0.5f;
    p->zupt_bias_mode = 1;  p->zupt_bias_min_s = 1.0f;  p->zupt_bias_max_s = 30.0f;  p->zupt_bias_floor = 3e-4f;
    p->zupt_bias_lag_s = 0.5f;
    p->use_enc_v = p->use_enc_w = p->use_mag = p->use_zupt = true;
    for (int i = 0; i < 3; i++) p->mag_scale[i] = 1.0f;
    p->mag_norm_ref = 0.0f;  p->mag_norm_tol = 0.15f;  p->mag_ref_s = 2.0f;
    p->mag_dip_tol = 0.03f;  p->mag_step_tol = 0.07f;  p->mag_relock_s = 20.0f;
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
    f->P[X_PX][X_PX] = f->P[X_PY][X_PY] = 1e-4f;
    f->P[X_YAW][X_YAW] = 1e-4f;
    f->P[X_VEL][X_VEL] = 0.01f;
    f->P[X_BG][X_BG]   = 1e-3f;
    f->P[X_BA][X_BA]   = 0.01f;
    f->P[X_EPS][X_EPS] = 1e-4f;
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
    float psi = f->x[X_YAW], v = f->x[X_VEL];
    float c = cosf(psi), s = sinf(psi);

    float F[NS][NS] = {{0}};
    for (int i = 0; i < NS; i++) F[i][i] = 1.0f;
    F[X_PX][X_YAW] = -v*s*dt;  F[X_PX][X_VEL] = c*dt;
    F[X_PY][X_YAW] =  v*c*dt;  F[X_PY][X_VEL] = s*dt;
    F[X_YAW][X_BG] = -dt;      F[X_VEL][X_BA] = -dt;

    f->x[X_PX]  += v*c*dt;
    f->x[X_PY]  += v*s*dt;
    f->x[X_YAW]  = wrap(psi + (wz - f->x[X_BG])*dt);
    f->x[X_VEL] += (a_fwd - f->x[X_BA])*dt;

    float T[NS][NS] = {{0}}, Pn[NS][NS];
    for (int i = 0; i < NS; i++) for (int j = 0; j < NS; j++)
        for (int k = 0; k < NS; k++) T[i][j] += F[i][k]*f->P[k][j];
    for (int i = 0; i < NS; i++) for (int j = 0; j < NS; j++) {
        float sum = 0;
        for (int k = 0; k < NS; k++) sum += T[i][k]*F[j][k];
        Pn[i][j] = sum;
    }
    Pn[X_PX][X_PX] += 1e-8f*dt;  Pn[X_PY][X_PY] += 1e-8f*dt;
    Pn[X_YAW][X_YAW] += p->sig_gyro*p->sig_gyro*dt*dt;
    Pn[X_VEL][X_VEL] += p->sig_accel*p->sig_accel*dt*dt;
    Pn[X_BG][X_BG]   += p->sig_bg*p->sig_bg*dt;
    Pn[X_BA][X_BA]   += p->sig_ba*p->sig_ba*dt;
    Pn[X_EPS][X_EPS] += 1e-10f*dt;
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
    if (S <= 0) { f->n_rej[sensor]++; return false; }
    if (y*y > f->p.gate*f->p.gate*S) {
        bool wheel = sensor == S_ENC_V || sensor == S_ENC_W;
        f->rej_t[sensor] += f->dt_cur;
        if (!(wheel && f->p.gate_recover_s > 0 && f->rej_t[sensor] > f->p.gate_recover_s)) { f->n_rej[sensor]++; return false; }
    }
    f->rej_t[sensor] = 0;
    float K[NS];
    for (int i = 0; i < NS; i++) K[i] = PH[i]/S;
    // Position is never measured, so P[x,y] and its correlations grow without bound on a long drive. A confident update
    // (e.g. ZUPT v=0) would then move x,y through those correlations - seen as a 1.3 km jump on real data. With
    // pos_consider set for a sensor, the gain rows for x,y are zeroed (Schmidt "consider" states): only integration moves position.
    if (f->p.pos_consider & (1u << sensor)) K[X_PX] = K[X_PY] = 0.0f;
    for (int i = 0; i < NS; i++) f->x[i] += K[i]*y;
    // Joseph form for a scalar update, valid for any gain: P -= K PH' + PH K' - S K K'
    for (int i = 0; i < NS; i++) for (int j = 0; j < NS; j++)
        f->P[i][j] -= K[i]*PH[j] + PH[i]*K[j] - S*K[i]*K[j];
    for (int i = 0; i < NS; i++) {                       // keep symmetric / positive
        for (int j = i+1; j < NS; j++) { float m = 0.5f*(f->P[i][j] + f->P[j][i]); f->P[i][j] = f->P[j][i] = m; }
        if (f->P[i][i] < 1e-12f) f->P[i][i] = 1e-12f;
    }
    f->x[X_YAW] = wrap(f->x[X_YAW]);
    f->n_ok[sensor]++;
    return true;
}

// move the oldest held gyro sample of the current stop into the stop's mean/variance accumulators
static void zb_count_oldest(Fusion *f) {
    float v = f->zb_ring[f->zb_head], d = f->zb_rdt[f->zb_head];
    f->zb_sum += v;  f->zb_sq += v*v;  f->zb_n++;  f->zb_t += d;
    f->zb_rt -= d;  f->zb_head = (f->zb_head + 1) % ZB_RING;  f->zb_cnt--;
}

void fusion_step(Fusion *f, const Sample *s) {
    FusionParams *p = &f->p;
    float dt = s->dt;
    if (dt <= 0 || dt > 0.5f) return;
    f->dt_cur = dt;

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

    // 4. ZUPT (stopped = wheels still AND gyro still, for zupt_s seconds)
    // detector works on ~0.25 s low-passed signals: encoder counts are coarse at 200 Hz, averaging resolves slow rotation
    float k = dt/0.25f;
    f->vLf += k*(s->vL - f->vLf);  f->vRf += k*(s->vR - f->vRf);
    f->wzf += k*((wz - f->x[X_BG]) - f->wzf);  f->aff += k*((a_fwd - f->x[X_BA]) - f->aff);
    bool still = fabsf(f->vLf) < p->zupt_v && fabsf(f->vRf) < p->zupt_v &&
                 fabsf(f->wzf) < p->zupt_w && fabsf(f->aff) < p->zupt_a;
    f->still_t = still ? f->still_t + dt : 0.0f;
    bool was_stationary = f->stationary;
    f->stationary = f->still_t >= p->zupt_s;
    if (p->use_zupt && f->stationary) {
        float H[NS] = {0};
        H[X_VEL] = 1;  update(f, S_ZUPT, 0.0f - f->x[X_VEL], H, p->r_zupt_v*p->r_zupt_v);
        if (p->zupt_bias_mode == 0) {                    // legacy: one gyro-bias update per sample
            memset(H, 0, sizeof(H)); H[X_BG] = -1;
            update(f, S_ZUPT, 0.0f - (wz - f->x[X_BG]), H, p->r_zupt_w*p->r_zupt_w);
        } else {                                         // batch: collect the stop, update once (below)
            // the detector notices motion ~0.25 s late, so the newest samples of a stop may already be turning:
            // hold the last zupt_bias_lag_s in a ring and only count samples once they are older than that
            if (f->zb_cnt == ZB_RING) zb_count_oldest(f); // ring full (lag longer than the ring): count the oldest now
            int w = (f->zb_head + f->zb_cnt) % ZB_RING;
            f->zb_ring[w] = wz;  f->zb_rdt[w] = dt;  f->zb_cnt++;  f->zb_rt += dt;
            while (f->zb_cnt > 0 && f->zb_rt - f->zb_rdt[f->zb_head] >= p->zupt_bias_lag_s) zb_count_oldest(f);
        }
        memset(H, 0, sizeof(H)); H[X_BA] = -1;
        update(f, S_ZUPT, 0.0f - (a_fwd - f->x[X_BA]), H, p->r_zupt_a*p->r_zupt_a);
    }
    // Batch gyro-bias update: per-sample updates treat every gyro sample as independent, so a 2 s stop looked like a
    // precise bias measurement, but the wobble of a stopped robot is slow and correlated. Use the stop's mean rate once,
    // with variance = spread/n + a floor for the correlated part, and only for stops of at least zupt_bias_min_s.
    bool stop_ended = was_stationary && !f->stationary;
    bool stop_long  = f->stationary && p->zupt_bias_max_s > 0 && f->zb_t >= p->zupt_bias_max_s;
    if (p->use_zupt && p->zupt_bias_mode != 0 && (stop_ended || stop_long)) {
        if (f->zb_t >= p->zupt_bias_min_s && f->zb_n > 1) {
            float mean = f->zb_sum / f->zb_n;
            float var = f->zb_sq / f->zb_n - mean*mean;  if (var < 0) var = 0;
            float H[NS] = {0}; H[X_BG] = 1;
            update(f, S_ZUPT, mean - f->x[X_BG], H, var / f->zb_n + p->zupt_bias_floor*p->zupt_bias_floor);
        }
        f->zb_sum = f->zb_sq = f->zb_t = 0.0f;  f->zb_n = 0;
        if (stop_ended) { f->zb_head = f->zb_cnt = 0;  f->zb_rt = 0.0f; }   // drop the (possibly moving) newest samples
    }

    // 5. wheel forward speed  (checks linear motion)
    if (p->use_enc_v) {
        float H[NS] = {0}; H[X_VEL] = 1;
        update(f, S_ENC_V, 0.5f*(s->vL + s->vR) - f->x[X_VEL], H, p->r_enc_v*p->r_enc_v);
    }
    // 6. wheel yaw rate (vR-vL)/b. Model: w_enc = (wz - bg) + eps*v/b, where eps = wheel-diameter mismatch.
    //    Checks the gyro, learns gyro bias (when still) and eps (when moving). Gated, so slip is rejected.
    if (p->use_enc_w) {
        float b = p->track_width, H[NS] = {0};
        H[X_BG] = -1;  H[X_EPS] = f->x[X_VEL]/b;  H[X_VEL] = f->x[X_EPS]/b;
        float w_enc = (s->vR - s->vL)/b;
        update(f, S_ENC_W, w_enc - ((wz - f->x[X_BG]) + f->x[X_EPS]*f->x[X_VEL]/b), H, p->r_enc_w*p->r_enc_w);
    }
    // 7. magnetometer heading (absolute, tilt-compensated), skipped when the field looks disturbed
    if (p->use_mag && s->has_mag) {
        float m[3];
        for (int i = 0; i < 3; i++) m[i] = (s->m[i] - p->mag_off[i])*p->mag_scale[i];
        float mn = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
        // field-strength reference: given, or the mean |m| of the samples accepted into the heading-reference window
        float nref = p->mag_norm_ref > 0 ? p->mag_norm_ref : (f->mag_n > 0 ? f->mag_norm_sum / f->mag_n : mn);
        bool field_ok = mn > 0 && fabsf(mn - nref) < p->mag_norm_tol*nref;
        if (!field_ok) {
            f->n_rej[S_MAG]++;
            if (f->mag_ref_set) f->mag_rej_time += dt;   // all rejections count toward relocking, not only dip/step ones
        }
        else {
            float md = m[0]*u[0] + m[1]*u[1] + m[2]*u[2];
            float h[3] = { m[0]-md*u[0], m[1]-md*u[1], m[2]-md*u[2] };
            float fw[3] = { 1.0f - u[0]*u[0], -u[0]*u[1], -u[0]*u[2] };       // body x, horizontal part
            float lf[3] = { u[1]*fw[2]-u[2]*fw[1], u[2]*fw[0]-u[0]*fw[2], u[0]*fw[1]-u[1]*fw[0] };  // u x fw = left
            float ang = atan2f(h[0]*lf[0]+h[1]*lf[1]+h[2]*lf[2], h[0]*fw[0]+h[1]*fw[1]+h[2]*fw[2]);
            if (!f->mag_ref_set) {                       // average (ang + yaw) over the reference window
                float r = ang + f->x[X_YAW];
                f->mag_sin += sinf(r); f->mag_cos += cosf(r);
                f->mag_dip_sum += md / mn;  f->mag_norm_sum += mn;  f->mag_n++;
                f->mag_t += dt;
                if (f->mag_t >= p->mag_ref_s) {
                    f->mag_ref = atan2f(f->mag_sin, f->mag_cos);
                    f->mag_dip_ref = f->mag_dip_sum / f->mag_n;
                    f->mag_ref_set = true;
                }
                return;
            }
            // disturbance checks beyond the field-strength test (a horizontal disturbance barely changes |m|)
            if (p->mag_dip_tol > 0 && fabsf(md/mn - f->mag_dip_ref) > p->mag_dip_tol) {
                f->n_rej[S_MAG]++; f->mag_rej_time += dt; return;
            }
            float psi_m = f->mag_ref - ang;
            float y = wrap(psi_m - f->x[X_YAW]);
            if (p->mag_step_tol > 0 && f->mag_innov_init && f->mag_rej_time < p->mag_relock_s &&
                fabsf(wrap(y - f->mag_innov_ema)) > p->mag_step_tol) {
                f->n_rej[S_MAG]++; f->mag_rej_time += dt; return;
            }
            float H[NS] = {0}; H[X_YAW] = 1;
            if (update(f, S_MAG, y, H, p->r_mag*p->r_mag)) {
                f->mag_rej_time = 0;
                if (!f->mag_innov_init) { f->mag_innov_ema = y; f->mag_innov_init = true; }
                else f->mag_innov_ema += (dt/2.0f) * wrap(y - f->mag_innov_ema);   // ~2 s time constant
            }
        }
    }
}
