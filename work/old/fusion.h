// fusion.h - portable 2D IMU + wheel-encoder + magnetometer fusion (no ESP-IDF deps)
// Frame: body x = forward, y = left, z = up. Yaw positive counter-clockwise.
// Units: accel m/s^2, gyro rad/s, wheel speeds m/s, mag any consistent unit.
#ifndef FUSION_H
#define FUSION_H
#include <stdbool.h>
#include <stdint.h>

#define NS 7
enum { PX, PY, YAW, VEL, BG, BA, EPS };   // state: x, y, yaw, forward speed, gyro-z bias, accel-fwd bias, wheel-scale mismatch (right vs left)
enum { S_ENC_V, S_ENC_W, S_MAG, S_ZUPT, S_COUNT };

typedef struct {
    // process noise
    float sig_gyro, sig_accel;       // rad/s, m/s^2   (white noise on the IMU inputs)
    float sig_bg, sig_ba;            // bias random walk (per sqrt(s))
    // measurement noise (std dev)
    float r_enc_v;                   // m/s   wheel forward speed
    float r_enc_w;                   // rad/s wheel-difference yaw rate
    float r_mag;                     // rad   magnetometer heading
    float r_zupt_v, r_zupt_w, r_zupt_a;
    float beta;                      // Madgwick gain: lower = trust gyro more for tilt (accel is corrupted when accelerating)
    float gate;                      // innovation gate, in sigmas (reject if |y| > gate*sqrt(S))
    float track_width;               // m, distance between wheels
    // ZUPT detector
    float zupt_v, zupt_w, zupt_a;    // thresholds on LOW-PASSED signals (m/s, rad/s, m/s^2)
    int   zupt_samples;              // consecutive samples needed
    // sensor switches (for the ablation study)
    bool use_enc_v, use_enc_w, use_mag, use_zupt;
    // magnetometer calibration + disturbance check
    float mag_off[3], mag_scale[3];
    float mag_norm_ref;              // 0 = learn from first sample
    float mag_norm_tol;              // fractional, e.g. 0.15
    int   mag_ref_samples;           // samples averaged to set the heading reference (robot should be still)
} FusionParams;

typedef struct {
    float a[3];       // m/s^2 specific force (includes gravity)
    float g[3];       // rad/s
    float m[3];       // magnetometer (raw)
    bool  has_mag;
    float vL, vR;     // wheel linear speeds, m/s
    float dt;         // s
} Sample;

typedef struct {
    float q[4], beta;
    float x[NS], P[NS][NS];
    float gbias0[3];         // startup gyro bias (3 axes)
    float mag_ref, mag_sin, mag_cos;   // heading offset (averaged over the first samples; one noisy sample would bias heading forever)
    int   mag_n;
    bool  mag_ref_set;
    int   still_count;
    bool  stationary;
    float vLf, vRf, wzf, aff;      // low-pass filtered signals for the ZUPT detector
    uint32_t n_ok[S_COUNT], n_rej[S_COUNT];   // diagnostics: accepted / gated-out updates
    float    last_innov[S_COUNT];
    FusionParams p;
} Fusion;

void fusion_default_params(FusionParams *p);
// a_mean (m/s^2) and g_mean (rad/s) averaged over a still period at startup.
// Pass g_mean = NULL if the robot was not still (gyro bias then starts at 0).
void fusion_init(Fusion *f, const FusionParams *p, const float a_mean[3], const float g_mean[3]);
void fusion_step(Fusion *f, const Sample *s);

#endif
