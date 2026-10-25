// main.c - ESP32 (ESP-IDF v5) firmware. Same fusion core as the PC replay (fusion.c).
// NOT compiled here (no ESP-IDF toolchain in my sandbox): build it yourself and report any errors.
// Assumed sensor mounting: MPU6050 x = forward, y = left, z = up. Otherwise edit imu_to_body().
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "fusion.h"
#include "calib.h"          // IMU calibration from imu_calib.py (IMU_CALIBRATED 0 = none yet)

// LOG_RAW 1 (build env "lograw"): stream raw samples over USB at 921600 baud for imu_calib.py / log_capture.py,
// and never drive the motors (the calibration session is done by hand).
#ifndef LOG_RAW
#define LOG_RAW 0
#endif

static const char *TAG = "APP";

// ---------- Pins & config ----------
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_INT_PIN 4
#define MPU_ADDR 0x68
#define L_MOTOR_IN1 26
#define L_MOTOR_IN2 27
#define L_MOTOR_PWM 25
#define R_MOTOR_IN1 14
#define R_MOTOR_IN2 15
#define R_MOTOR_PWM 13
#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 32
#define R_ENC_B 33

#define COUNTS_PER_REV   (7 * 100 * 4)
#define WHEEL_DIAMETER   0.065f      // m  -> calibrate: drive a measured distance, scale this
#define TRACK_WIDTH      0.15f       // m  -> measure centre-to-centre of the wheels
#define L_ENC_SIGN       (+1)        // flip to -1 if a wheel counts negative when driving forward
#define R_ENC_SIGN       (+1)
#define METERS_PER_COUNT ((3.14159265f * WHEEL_DIAMETER) / COUNTS_PER_REV)
#define STILL_SAMPLES    400         // robot MUST be motionless this long at boot (2 s)

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f
#define DEG2RAD     0.01745329f
#define G0          9.81f

#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE 0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B

typedef struct { int in1, in2, pwm; ledc_channel_t ch; } Motor_t;

static Motor_t motorL, motorR;
static TaskHandle_t imu_task_handle;
static i2c_master_dev_handle_t mpu;
static pcnt_unit_handle_t pcntL, pcntR;
static Fusion fus;
static volatile bool fusion_ready = false;

// ---------------- motors ----------------
static void motor_init_hw(Motor_t *m, ledc_channel_t ch) {
    m->ch = ch;
    gpio_config_t io = { .pin_bit_mask = (1ULL << m->in1) | (1ULL << m->in2), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    ledc_channel_config_t c = { .gpio_num = m->pwm, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = ch,
                                .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&c);
}
static void motor_set(Motor_t *m, int duty, bool fwd) {
    gpio_set_level(m->in1, fwd ? 1 : 0);
    gpio_set_level(m->in2, fwd ? 0 : 1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->ch);
}

// ---------------- encoders (quadrature, x4) ----------------
static void encoder_init_unit(pcnt_unit_handle_t *unit, int a, int b) {
    pcnt_unit_config_t uc = { .high_limit = 32767, .low_limit = -32768, .flags.accum_count = true };
    pcnt_new_unit(&uc, unit);
    pcnt_glitch_filter_config_t flt = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(*unit, &flt);
    pcnt_chan_config_t ca = { .edge_gpio_num = a, .level_gpio_num = b };
    pcnt_channel_handle_t cha; pcnt_new_channel(*unit, &ca, &cha);
    pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_chan_config_t cb = { .edge_gpio_num = b, .level_gpio_num = a };
    pcnt_channel_handle_t chb; pcnt_new_channel(*unit, &cb, &chb);
    pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_add_watch_point(*unit, 32767);          // accum_count only accumulates across these limits
    pcnt_unit_add_watch_point(*unit, -32768);
    pcnt_unit_enable(*unit); pcnt_unit_clear_count(*unit); pcnt_unit_start(*unit);
}

// ---------------- MPU6050 ----------------
static void mpu_write(uint8_t reg, uint8_t v) { uint8_t b[2] = { reg, v }; i2c_master_transmit(mpu, b, 2, 100); }
static esp_err_t mpu_read(uint8_t reg, uint8_t *d, size_t n) { return i2c_master_transmit_receive(mpu, &reg, 1, d, n, 100); }
static void IRAM_ATTR mpu_isr(void *arg) {
    BaseType_t w = pdFALSE; vTaskNotifyGiveFromISR(imu_task_handle, &w); if (w) portYIELD_FROM_ISR();
}

// MPU6050 has NO magnetometer. Use an MPU9250/AK8963, QMC5883L or HMC5883L on the same I2C bus and
// fill this in (return true + m[3] in any consistent unit, axes in the SAME x-fwd,y-left,z-up frame as the IMU).
static bool mag_read(float m[3]) { (void)m; return false; }

// sensor-frame calibration (bias, scale, misalignment): calibrated = C * (raw - b). The gyro bias in calib.h is the one
// measured during calibration; the boot still period below re-measures what is left of it at every power-up.
static void imu_calibrate(float a[3], float g[3]) {
#if IMU_CALIBRATED
    float ta[3], tg[3];
    for (int i = 0; i < 3; i++) { ta[i] = a[i] - CAL_ACC_B[i];  tg[i] = g[i] - CAL_GYR_B[i]; }
    for (int i = 0; i < 3; i++) {
        a[i] = CAL_ACC_C[i][0]*ta[0] + CAL_ACC_C[i][1]*ta[1] + CAL_ACC_C[i][2]*ta[2];
        g[i] = CAL_GYR_C[i][0]*tg[0] + CAL_GYR_C[i][1]*tg[1] + CAL_GYR_C[i][2]*tg[2];
    }
#else
    (void)a; (void)g;
#endif
}

// remap sensor axes to body frame here if the board is mounted differently
static void imu_to_body(const float a_in[3], const float g_in[3], float a[3], float g[3]) {
    for (int i = 0; i < 3; i++) { a[i] = a_in[i]; g[i] = g_in[i]; }
}

static void imu_task(void *arg) {
    uint8_t d[14];
    int n_still = 0; float a_sum[3] = {0}, g_sum[3] = {0}, a_sq[3] = {0}, g_sq[3] = {0};
    int lastL = 0, lastR = 0; int64_t last_us = esp_timer_get_time(); int tick = 0;
    FusionParams p;
    fusion_default_params(&p);
    p.track_width = TRACK_WIDTH;
    // Real-world profile: tuned jointly on NCLT 2012-01-08 + 2012-02-04 (Segway, 12 km, first 40 min of each; rest held out).
    // Platform-specific values (wheel scale, track width, mag offsets) are NOT copied - calibrate them on this robot.
    p.use_enc_w      = false;    // (vR-vL)/b vs gyro hurt on real data (slip, pivot turns)
    p.gate           = 8.0f;     // 3 sigma rejected too many good wheel/mag samples on real sensors
    p.gate_recover_s = 1.0f;     // accept wheels after 1 s of continuous rejection (no permanent lock-out)
    p.r_enc_v        = 0.06f;
    p.sig_bg         = 3e-5f;    // gyro-bias random walk
    p.beta           = 0.01f;    // Madgwick: trust gyro more for tilt
    p.zupt_w         = 0.005f;   // rad/s, low-passed: slow pivot turns otherwise look 'still' and corrupt the gyro bias
    p.zupt_s         = 8.4f;     // s still before ZUPT (short stops gave noisy bias updates)
    //   p.zupt_v ~ wheel-speed noise when stopped (one encoder count per sample = 0.0146 m/s at 200 Hz here)
    // magnetometer (only once mag_read() is implemented):
    p.mag_ref_s      = 250.0f;   // average the heading reference over ~4 min of driving, not the (often disturbed) start spot
    p.r_mag          = 7.0f;     // rad, per sample at ~47 Hz: weak per-sample pull, strong on average. Scale ~sqrt(rate/47) if faster
    p.mag_norm_tol   = 0.15f;  p.mag_dip_tol = 0.07f;  p.mag_step_tol = 0.6f;
    p.use_mag = false;           // mag_read() is a stub
    // p.mag_off / p.mag_scale: fill in from your magnetometer calibration (rotate the robot slowly through all orientations)

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (mpu_read(REG_ACCEL_XOUT_H, d, 14) != ESP_OK) continue;
        int64_t now = esp_timer_get_time(); float dt = (now - last_us) * 1e-6f; last_us = now;

        float araw[3] = { (int16_t)((d[0]<<8)|d[1]) / ACCEL_SCALE * G0,
                          (int16_t)((d[2]<<8)|d[3]) / ACCEL_SCALE * G0,
                          (int16_t)((d[4]<<8)|d[5]) / ACCEL_SCALE * G0 };
        float graw[3] = { (int16_t)((d[8]<<8)|d[9])   / GYRO_SCALE * DEG2RAD,    // deg/s -> rad/s (old code skipped this)
                          (int16_t)((d[10]<<8)|d[11]) / GYRO_SCALE * DEG2RAD,
                          (int16_t)((d[12]<<8)|d[13]) / GYRO_SCALE * DEG2RAD };
        imu_calibrate(araw, graw);
        float a[3], g[3]; imu_to_body(araw, graw, a, g);

        int cL, cR; pcnt_unit_get_count(pcntL, &cL); pcnt_unit_get_count(pcntR, &cR);
        float vL = L_ENC_SIGN * (cL - lastL) * METERS_PER_COUNT / dt;
        float vR = R_ENC_SIGN * (cR - lastR) * METERS_PER_COUNT / dt;
        lastL = cL; lastR = cR;
#if LOG_RAW
        // raw sensor-frame counts (+-2 g, +-250 dps) and encoder counts: t_us,ax,ay,az,gx,gy,gz,cL,cR
        printf("%lld,%d,%d,%d,%d,%d,%d,%d,%d\n", (long long)now,
               (int16_t)((d[0]<<8)|d[1]), (int16_t)((d[2]<<8)|d[3]), (int16_t)((d[4]<<8)|d[5]),
               (int16_t)((d[8]<<8)|d[9]), (int16_t)((d[10]<<8)|d[11]), (int16_t)((d[12]<<8)|d[13]), cL, cR);
        continue;
#endif

        if (!fusion_ready) {                      // boot calibration: average a still period
            for (int i = 0; i < 3; i++) { a_sum[i] += a[i]; g_sum[i] += g[i]; a_sq[i] += a[i]*a[i]; g_sq[i] += g[i]*g[i]; }
            if (++n_still >= STILL_SAMPLES) {
                // the robot must really have been still: a bump during boot would bake a wrong gyro bias and tilt
                // into the whole run. Same limits as imu_live's static check (gyro std 0.02 rad/s, accel 0.3 m/s^2).
                float g_std = 0, a_std = 0;
                for (int i = 0; i < 3; i++) {
                    float ma = a_sum[i] / n_still, mg = g_sum[i] / n_still;
                    a_std = fmaxf(a_std, sqrtf(fmaxf(0.0f, a_sq[i] / n_still - ma*ma)));
                    g_std = fmaxf(g_std, sqrtf(fmaxf(0.0f, g_sq[i] / n_still - mg*mg)));
                }
                if (g_std > 0.02f || a_std > 0.3f) {
                    ESP_LOGW(TAG, "moved during boot calibration (gyro std %.3f, accel std %.2f): keep still, retrying", g_std, a_std);
                } else {
                    for (int i = 0; i < 3; i++) { a_sum[i] /= n_still; g_sum[i] /= n_still; }
                    fusion_init(&fus, &p, a_sum, g_sum);
                    fusion_ready = true;
                    ESP_LOGI(TAG, "calibrated: gyro bias %.4f %.4f %.4f rad/s", g_sum[0], g_sum[1], g_sum[2]);
                }
                n_still = 0;
                for (int i = 0; i < 3; i++) { if (!fusion_ready) { a_sum[i] = g_sum[i] = 0; } a_sq[i] = g_sq[i] = 0; }
            }
            continue;
        }
        Sample s = {0}; memcpy(s.a, a, sizeof a); memcpy(s.g, g, sizeof g);
        s.has_mag = mag_read(s.m); s.vL = vL; s.vR = vR; s.dt = dt;
        fusion_step(&fus, &s);

        if (++tick >= 200) {
            tick = 0;
            ESP_LOGI(TAG, "x %.3f y %.3f yaw %.1fdeg v %.3f bg %.4f eps %.4f still %d rej(v/w/mag) %u/%u/%u",
                     fus.x[X_PX], fus.x[X_PY], fus.x[X_YAW] * 57.2958f, fus.x[X_VEL], fus.x[X_BG], fus.x[X_EPS], fus.stationary,
                     (unsigned)fus.n_rej[S_ENC_V], (unsigned)fus.n_rej[S_ENC_W], (unsigned)fus.n_rej[S_MAG]);
        }
    }
}

static void imu_init(void) {
    i2c_master_bus_config_t bc = { .i2c_port = I2C_NUM_0, .sda_io_num = I2C_SDA, .scl_io_num = I2C_SCL,
                                   .clk_source = I2C_CLK_SRC_DEFAULT, .flags.enable_internal_pullup = true };
    i2c_master_bus_handle_t bus; i2c_new_master_bus(&bc, &bus);
    i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = MPU_ADDR, .scl_speed_hz = 400000 };
    i2c_master_bus_add_device(bus, &dc, &mpu);
    xTaskCreate(imu_task, "imu", 6144, NULL, 6, &imu_task_handle);   // task exists before the ISR can fire
    mpu_write(REG_PWR_MGMT_1, 0x00); vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write(REG_CONFIG, 0x03);          // DLPF 44 Hz -> gyro rate 1 kHz
    mpu_write(REG_SMPLRT_DIV, 4);         // 1000/(1+4) = 200 Hz
    mpu_write(REG_INT_PIN_CFG, 0x10);
    mpu_write(REG_INT_ENABLE, 0x01);
    gpio_config_t io = { .pin_bit_mask = 1ULL << MPU_INT_PIN, .mode = GPIO_MODE_INPUT,
                         .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_POSEDGE };
    gpio_config(&io); gpio_install_isr_service(0); gpio_isr_handler_add(MPU_INT_PIN, mpu_isr, NULL);
}

// motors only start AFTER the still calibration finishes
static void drive_task(void *arg) {
    while (!fusion_ready) vTaskDelay(pdMS_TO_TICKS(50));
    while (1) { motor_set(&motorL, 150, true); motor_set(&motorR, 150, true); vTaskDelay(pdMS_TO_TICKS(100)); }
}

void app_main(void) {
    ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
                              .timer_num = LEDC_TIMER_0, .freq_hz = 20000, .clk_cfg = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    motorL = (Motor_t){ L_MOTOR_IN1, L_MOTOR_IN2, L_MOTOR_PWM, LEDC_CHANNEL_0 };
    motorR = (Motor_t){ R_MOTOR_IN1, R_MOTOR_IN2, R_MOTOR_PWM, LEDC_CHANNEL_1 };
    motor_init_hw(&motorL, LEDC_CHANNEL_0); motor_init_hw(&motorR, LEDC_CHANNEL_1);
    encoder_init_unit(&pcntL, L_ENC_A, L_ENC_B); encoder_init_unit(&pcntR, R_ENC_A, R_ENC_B);
#if LOG_RAW
    uart_set_baudrate(UART_NUM_0, 921600);   // 200 Hz x ~50 chars does not fit in 115200 baud
    printf("t_us,ax,ay,az,gx,gy,gz,cL,cR\n");
    imu_init();                              // no drive task: motors stay off while logging
    (void)drive_task;
#else
    imu_init();
    xTaskCreate(drive_task, "drive", 2048, NULL, 4, NULL);
#endif
}
