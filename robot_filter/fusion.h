// fusion.h - portable 2D IMU + wheel-encoder + magnetometer fusion (no ESP-IDF deps)
// Frame: body x = forward, y = left, z = up. Yaw positive counter-clockwise.
// Units: accel m/s^2, gyro rad/s, wheel speeds m/s, mag any consistent unit.
#ifndef FUSION_H
#define FUSION_H
#include <stdbool.h>
#include <stdint.h>

#define NS 7
#define ZB_RING 256   // >= zupt_bias_lag_s * sample rate (0.5 s at 200 Hz = 100)
enum { X_PX, X_PY, X_YAW, X_VEL, X_BG, X_BA, X_EPS };   // state: x, y, yaw, forward speed, gyro-z bias, accel-fwd bias, wheel-scale mismatch (right vs left)
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
    uint8_t pos_consider;            // bit per sensor (1<<S_ENC_V, 1<<S_ENC_W, 1<<S_MAG, 1<<S_ZUPT): that sensor's updates
                                     //   never move x,y directly (no position sensor; see update())
    float gate_recover_s;            // s: wheel updates rejected continuously this long are accepted anyway (0 = never). Stops a
                                     //    gate lock-out when the state drifted away from the wheels (slip bursts are shorter than this)
    float track_width;               // m, distance between wheels
    // ZUPT detector
    float zupt_v, zupt_w, zupt_a;    // thresholds on LOW-PASSED signals (m/s, rad/s, m/s^2)
    float zupt_s;                    // s the detector must stay 'still' before ZUPT applies (seconds, so it is rate-independent)
    int   zupt_bias_mode;            // gyro bias from stops: 0 = update every sample (legacy), 1 = one update per stop (mean)
                                     //   mode 1 validated on NCLT training sessions + sim (robot_filter/README.md)
    float zupt_bias_min_s;           // s: shorter stops do not touch the gyro bias (mode 1)
    float zupt_bias_max_s;           // s: on a long stop, update every this many seconds (mode 1)
    float zupt_bias_floor;           // rad/s: min std of a stop's mean rate (correlated wobble, mode 1)
    float zupt_bias_lag_s;           // s: newest samples of a stop left out (detector lag; mode 1)
    // sensor switches (for the ablation study)
    bool use_enc_v, use_enc_w, use_mag, use_zupt;
    // magnetometer calibration + disturbance check
    float mag_off[3], mag_scale[3];
    float mag_norm_ref;              // 0 = learn: average |m| over the heading-reference window
    float mag_norm_tol;              // fractional, e.g. 0.15
    float mag_ref_s;                 // s of mag averaged (with the gyro-propagated yaw) to set the heading reference. Longer
                                     //    = local disturbances at the start position average out; the gyro carries yaw meanwhile
    float mag_dip_tol;               // max change of sin(dip angle) vs the startup value (0 = off): catches horizontal disturbances
    float mag_step_tol;              // rad: a sudden jump of mag-vs-filter heading error bigger than this is a disturbance (0 = off)
    float mag_relock_s;              // s: accept the magnetometer again after rejecting this long (heading may have drifted)
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
    float mag_t, mag_norm_sum;         // time and |m| sum over the reference window
    float mag_dip_sum, mag_dip_ref;    // sin(dip) = (m.up)/|m|, averaged over the same startup samples
    float mag_innov_ema; bool mag_innov_init;   // slow average of the mag-vs-filter heading error (accepted samples only)
    float mag_rej_time;                // time spent continuously rejecting
    bool  mag_ref_set;
    float still_t;
    bool  stationary;
    float vLf, vRf, wzf, aff;      // low-pass filtered signals for the ZUPT detector
    float zb_sum, zb_sq, zb_t; int zb_n;   // gyro-rate accumulator over the current stop (batch bias update)
    float zb_ring[ZB_RING], zb_rdt[ZB_RING], zb_rt; int zb_head, zb_cnt;   // last zupt_bias_lag_s of samples, not yet counted
    uint32_t n_ok[S_COUNT], n_rej[S_COUNT];   // diagnostics: accepted / gated-out updates
    float    last_innov[S_COUNT];
    float    rej_t[S_COUNT];         // time spent continuously rejecting each sensor
    float    dt_cur;
    FusionParams p;
} Fusion;

void fusion_default_params(FusionParams *p);
// a_mean (m/s^2) and g_mean (rad/s) averaged over a still period at startup.
// Pass g_mean = NULL if the robot was not still (gyro bias then starts at 0).
void fusion_init(Fusion *f, const FusionParams *p, const float a_mean[3], const float g_mean[3]);
void fusion_step(Fusion *f, const Sample *s);

#endif
