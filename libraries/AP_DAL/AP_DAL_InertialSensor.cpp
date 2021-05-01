#include "AP_DAL_InertialSensor.h"

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

bool dal_enable_random;
float dal_random_scale;
int32_t dal_random_seed;

AP_DAL_InertialSensor::AP_DAL_InertialSensor()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        _RISI[i].instance = i;
    }
    const char *seed_env = getenv("DAL_SEED");
    if (seed_env) {
        dal_random_seed = int32_t(atol(seed_env));
        srandom(unsigned(dal_random_seed));
        dal_enable_random = true;
    }
    if (dal_enable_random) {
        const char *seed_scale = getenv("DAL_RANDOM_SCALE");
        if (seed_scale) {
            dal_random_scale = atof(seed_scale);
        }
    }
}

void AP_DAL_InertialSensor::start_frame()
{
    const auto &ins = AP::ins();

    const log_RISH old_RISH = _RISH;

    _RISH.loop_rate_hz = ins.get_loop_rate_hz();
    _RISH.primary_gyro = ins.get_primary_gyro();
    _RISH.loop_delta_t = ins.get_loop_delta_t();
    _RISH.primary_accel = ins.get_primary_accel();
    _RISH.accel_count = ins.get_accel_count();
    _RISH.gyro_count = ins.get_gyro_count();
    WRITE_REPLAY_BLOCK_IFCHANGED(RISH, _RISH, old_RISH);

    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        log_RISI &RISI = _RISI[i];
        const log_RISI old_RISI = RISI;

        // accel data
        RISI.use_accel = ins.use_accel(i);
        if (RISI.use_accel) {
            RISI.get_delta_velocity_ret = ins.get_delta_velocity(i, RISI.delta_velocity, RISI.delta_velocity_dt);
        }

        // gryo data
        RISI.use_gyro = ins.use_gyro(i);
        if (RISI.use_gyro) {
            RISI.get_delta_angle_ret = ins.get_delta_angle(i, RISI.delta_angle, RISI.delta_angle_dt);
        }

        update_filtered(i);

        WRITE_REPLAY_BLOCK_IFCHANGED(RISI, RISI, old_RISI);

        // update sensor position
        pos[i] = ins.get_imu_pos_offset(i);
    }
}

// update filtered gyro and accel
void AP_DAL_InertialSensor::update_filtered(uint8_t i)
{
    if (!is_positive(alpha)) {
        // we use a constant 20Hz for EKF filtered accel/gyro, making the EKF
        // independent of the INS filter settings
        const float cutoff_hz = 20.0;
        alpha = calc_lowpass_alpha_dt(get_loop_delta_t(), cutoff_hz);
    }
    if (is_positive(_RISI[i].delta_angle_dt)) {
        gyro_filtered[i] += ((_RISI[i].delta_angle/_RISI[i].delta_angle_dt) - gyro_filtered[i]) * alpha;
    }
    if (is_positive(_RISI[i].delta_velocity_dt)) {
        accel_filtered[i] += ((_RISI[i].delta_velocity/_RISI[i].delta_velocity_dt) - accel_filtered[i]) * alpha;
    }
}

void AP_DAL_InertialSensor::handle_message(const log_RISI &msg)
{
    _RISI[msg.instance] = msg;
    if (dal_enable_random) {
        auto &r = _RISI[msg.instance];
        r.delta_velocity += rand_vec3f() * dal_random_scale;
        r.delta_angle += rand_vec3f() * dal_random_scale;
    }
    pos[msg.instance] = AP::ins().get_imu_pos_offset(msg.instance);
    update_filtered(msg.instance);
}
