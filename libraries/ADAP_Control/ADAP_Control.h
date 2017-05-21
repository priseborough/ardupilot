#pragma once

#include <AP_Common/AP_Common.h>
#include <DataFlash/DataFlash.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>
#include <DataFlash/DataFlash.h>

class ADAP_Control {
public:
	ADAP_Control(const char *_log_msg_name) :
        log_msg_name(_log_msg_name)
    { 
		AP_Param::setup_object_defaults(this, var_info);
	}

    // calculate new output value
    float update(uint16_t loop_rate_hz, float target_rate, float sensor_rate, float scaler, float aspeed);
    
    void adaptive_tuning_send(mavlink_channel_t chan, uint8_t pid_axis);

    // setup optional PID_Info structure
    void set_pid_info(DataFlash_Class::PID_Info *_pid_info) {
        pid_info = _pid_info;
    }
    
	static const struct AP_Param::GroupInfo var_info[];

    // return true if the controller is enabled
    bool enabled(void) const;
    
private:

    const char *log_msg_name;

    // user settable parameters
    AP_Int8  enable_chan;
    AP_Float alpha;
    AP_Float gamma_theta;
    AP_Float gamma_omega;
    AP_Float gamma_sigma;
    AP_Float theta_max;
    AP_Float theta_min;
    AP_Float theta_epsilon;
    AP_Float omega_max;
    AP_Float omega_min;
    AP_Float omega_epsilon;
    AP_Float sigma_max;
    AP_Float sigma_min;
    AP_Float sigma_epsilon;
    AP_Float w0;
    AP_Float k;
    AP_Float kg;

    // internal state
    uint64_t last_run_us;
    float r;
    float x;
    float x_error;
    float eta;
    float integrator;
    float theta;
    float omega;
    float sigma;
    float u;
    float u_lowpass;
    float u_sp;
    float x_m;
    float theta_dot;
    float omega_dot;
    float sigma_dot;
    float out1;
    float theta1;
    float omega1;
    float sigma1;
    float f;
    float f_dot;
    
	LowPassFilter2pFloat r_filter;
        LowPassFilter2pFloat u_filter;

    // optional PID_Info structure
    DataFlash_Class::PID_Info *pid_info;
    
    void reset(uint16_t loop_rate_hz);
    float projection_operator(float theta, float y, float epsilon, float theta_max, float theta_min) const;
    float trapezoidal_integration(float y0, float y1_dot, float dt, float &y0_dot);
};
