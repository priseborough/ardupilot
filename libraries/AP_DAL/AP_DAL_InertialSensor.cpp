#include "AP_DAL_InertialSensor.h"

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
bool dal_enable_random;
int32_t dal_random_seed;
#endif

AP_DAL_InertialSensor::AP_DAL_InertialSensor()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RISI); i++) {
        _RISI[i].instance = i;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const char *seed_env = getenv("DAL_SEED");
    if (seed_env) {
        dal_random_seed = int32_t(atol(seed_env));
        srandom(unsigned(dal_random_seed));
        dal_enable_random = true;
    }
    if (dal_enable_random) {
         // spec is for axis to axis misalignment so halve individual axis variance
        const float ang_offset = radians(0.25f) / sqrtf(2.0f);

        // Spec says +-0.2% nonlinearity, but is vague about how it is measured.
         const float scale_error = 0.2E-2f;
        sensor_error_params.TransferMatrix = Matrix3f(Vector3f(1.0f + scale_error*rand_float(), ang_offset*rand_ndist(), ang_offset*rand_ndist()),
                                                      Vector3f(ang_offset*rand_ndist(), 1.0f + scale_error*rand_float(), ang_offset*rand_ndist()),
                                                      Vector3f(ang_offset*rand_ndist(), ang_offset*rand_ndist(), 1.0f + scale_error*rand_float()));
printf("Tmat = [[%e,%e,%e]\n",(double)sensor_error_params.TransferMatrix.a.x,(double)sensor_error_params.TransferMatrix.a.y,(double)sensor_error_params.TransferMatrix.a.z);
printf("        [%e,%e,%e]\n",(double)sensor_error_params.TransferMatrix.b.x,(double)sensor_error_params.TransferMatrix.b.y,(double)sensor_error_params.TransferMatrix.b.z);
printf("        [%e,%e,%e]]\n",(double)sensor_error_params.TransferMatrix.c.x,(double)sensor_error_params.TransferMatrix.c.y,(double)sensor_error_params.TransferMatrix.c.z);

        // Noise values taken from Allan deviation plot for 1 second integration interval
        // This will be adjsuted for IMU integration period when it is applied
        sensor_error_params.RateNoise = Vector3f(radians(8.0f), radians(8.0f), radians(11.0f)); // rad/hour
        sensor_error_params.RateNoise /= 3600.0f; // rad/sec
printf("Rnoise = [%e,%e,%e]\n",(double)sensor_error_params.RateNoise.x,(double)sensor_error_params.RateNoise.y,(double)sensor_error_params.RateNoise.z);

        // assume sensitivity of each gyro to acceleration as specified is the total accel, not per axis, diveide by sqrt(3)
        Vector3f Kacc = Vector3f(radians(0.572E-3f), radians(1.02E-3f), radians(0.408E-3f));
        Kacc /= sqrtf(3.0f);
        sensor_error_params.AccelToRateMatrix = Matrix3f(Vector3f( Kacc.x*rand_ndist(), Kacc.y*rand_ndist(), Kacc.z*rand_ndist()),
                                                         Vector3f( Kacc.x*rand_ndist(), Kacc.y*rand_ndist(), Kacc.z*rand_ndist()),
                                                         Vector3f( Kacc.x*rand_ndist(), Kacc.y*rand_ndist(), Kacc.z*rand_ndist()));
printf("AtoRmat = [[%e,%e,%e]\n",(double)sensor_error_params.AccelToRateMatrix.a.x,(double)sensor_error_params.AccelToRateMatrix.a.y,(double)sensor_error_params.AccelToRateMatrix.a.z);
printf("           [%e,%e,%e]\n",(double)sensor_error_params.AccelToRateMatrix.b.x,(double)sensor_error_params.AccelToRateMatrix.b.y,(double)sensor_error_params.AccelToRateMatrix.b.z);
printf("           [%e,%e,%e]]\n",(double)sensor_error_params.AccelToRateMatrix.c.x,(double)sensor_error_params.AccelToRateMatrix.c.y,(double)sensor_error_params.AccelToRateMatrix.c.z);

    }
#endif // HAL_BOARD_SITL
}

float AP_DAL_InertialSensor::rand_ndist() {
    float ret = 0.0f;
    for (uint8_t i=0; i<5; i++) {
        ret = ret + rand_float();
    }
    ret /= sqrtf(5.0f);
    return ret;
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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (dal_enable_random) {
        auto &r = _RISI[msg.instance];
        // adjust rate noise 1-STD for integration interval as well as converting to a delta angle
        // hence the use of dt^(3/2)
        Vector3f delta_angle_from_noise = sensor_error_params.RateNoise * powf(r.delta_angle_dt, 1.5f);
        delta_angle_from_noise.x *= rand_ndist();
        delta_angle_from_noise.y *= rand_ndist();
        delta_angle_from_noise.z *= rand_ndist();
        // apply misalignment and scale factor to 'truth data' before adding accel and noise effects
        r.delta_angle = sensor_error_params.TransferMatrix * r.delta_angle + delta_angle_from_noise;
        if (r.delta_velocity_dt > 0.0f) {
            const Vector3f accel = r.delta_velocity / r.delta_velocity_dt;
            const Vector3f angle_rate_from_accel = sensor_error_params.AccelToRateMatrix * accel;
            const Vector3f delta_angle_from_accel = angle_rate_from_accel * r.delta_angle_dt;
            r.delta_angle  += delta_angle_from_accel;
        }
    }
#endif // HAL_BOARD_SITL
    pos[msg.instance] = AP::ins().get_imu_pos_offset(msg.instance);
    update_filtered(msg.instance);
}
