/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simple electric motor simulator class
*/

#include "SIM_Motor.h"
#include <AP_Motors/AP_Motors.h>

using namespace SITL;

// calculate rotational accel and thrust for a motor
void Motor::calculate_forces(const struct sitl_input &input,
                             uint8_t motor_offset,
                             Vector3f &rot_accel,
                             Vector3f &thrust,
                             const Vector3f &velocity_air_bf,
                             float air_density,
                             float voltage)
{
    // fudge factors
    const float yaw_scale = radians(400);

    const float pwm = input.servos[motor_offset+servo];
    float command = pwm_to_command(pwm);
    float voltage_scale = voltage / voltage_max;

    if (voltage_scale < 0.1) {
        // battery is dead
        rot_accel.zero();
        thrust.zero();
        current = 0;
        return;
    }

    // apply slew limiter to command
    uint64_t now_us = AP_HAL::micros64();
    if (last_calc_us != 0 && slew_max > 0) {
        float dt = (now_us - last_calc_us)*1.0e-6;
        float slew_max_change = slew_max * dt;
        command = constrain_float(command, last_command-slew_max_change, last_command+slew_max_change);
    }
    last_calc_us = now_us;
    last_command = command;

    // the yaw torque of the motor
    Vector3f rotor_torque(0, 0, yaw_factor * command * yaw_scale * voltage_scale);

    // calculate velocity into prop, clipping at zero, assumes zero roll/pitch
    const float velocity_in = MAX(0, -velocity_air_bf.z);

    // get thrust for untilted motor
    const float motor_thrust = calc_thrust(command, air_density, velocity_in);

    // get power for untilted motor
    const float motor_power = calc_power(motor_thrust, air_density, velocity_in);

    // thrust in NED
    thrust = {0, 0, -motor_thrust};

    // define the arm position relative to center of mass
    Vector3f arm(cosf(radians(angle)), sinf(radians(angle)), 0);
    arm *= diagonal_size;

    // work out roll and pitch of motor relative to it pointing straight up
    float roll = 0, pitch = 0;

    uint64_t now = AP_HAL::micros64();
    
    // possibly roll and/or pitch the motor
    if (roll_servo >= 0) {
        uint16_t servoval = update_servo(input.servos[roll_servo+motor_offset], now, last_roll_value);
        if (roll_min < roll_max) {
            roll = constrain_float(roll_min + (servoval-1000)*0.001*(roll_max-roll_min), roll_min, roll_max);
        } else {
            roll = constrain_float(roll_max + (2000-servoval)*0.001*(roll_min-roll_max), roll_max, roll_min);
        }
    }
    if (pitch_servo >= 0) {
        uint16_t servoval = update_servo(input.servos[pitch_servo+motor_offset], now, last_pitch_value);
        if (pitch_min < pitch_max) {
            pitch = constrain_float(pitch_min + (servoval-1000)*0.001*(pitch_max-pitch_min), pitch_min, pitch_max);
        } else {
            pitch = constrain_float(pitch_max + (2000-servoval)*0.001*(pitch_min-pitch_max), pitch_max, pitch_min);
        }
    }
    last_change_usec = now;

    // possibly rotate the thrust vector and the rotor torque
    if (!is_zero(roll) || !is_zero(pitch)) {
        Matrix3f rotation;
        rotation.from_euler(radians(roll), radians(pitch), 0);
        thrust = rotation * thrust;
        rotor_torque = rotation * rotor_torque;
    }

    // calculate torque in newton-meters
    Vector3f torque = (arm % thrust) + rotor_torque;

    // calculate total rotational acceleration
    rot_accel.x = torque.x / moment_of_inertia.x;
    rot_accel.y = torque.y / moment_of_inertia.y;
    rot_accel.z = torque.z / moment_of_inertia.z;

    // calculate current
    current = motor_power / MAX(voltage, 0.1);
}

/*
  update and return current value of a servo. Calculated as 1000..2000
 */
uint16_t Motor::update_servo(uint16_t demand, uint64_t time_usec, float &last_value)
{
    if (servo_rate <= 0) {
        return demand;
    }
    if (servo_type == SERVO_RETRACT) {
        // handle retract servos
        if (demand > 1700) {
            demand = 2000;
        } else if (demand < 1300) {
            demand = 1000;
        } else {
            demand = last_value;
        }
    }
    demand = constrain_int16(demand, 1000, 2000);
    float dt = (time_usec - last_change_usec) * 1.0e-6f;
    // assume servo moves through 90 degrees over 1000 to 2000
    float max_change = 1000 * (dt / servo_rate) * 60.0f / 90.0f;
    last_value = constrain_float(demand, last_value-max_change, last_value+max_change);
    return uint16_t(last_value+0.5);
}


// calculate current and voltage
float Motor::get_current(void) const
{
    return current;
}

// setup PWM ranges for this motor
void Motor::setup_params(uint16_t _pwm_min, uint16_t _pwm_max, float _spin_min, float _spin_max, float _expo, float _slew_max,
                         float _vehicle_mass, float _diagonal_size, float _prop_fom, float _voltage_max, float _thrust_max,
                         float _prop_area)
{
    mot_pwm_min = _pwm_min;
    mot_pwm_max = _pwm_max;
    mot_spin_min = _spin_min;
    mot_spin_max = _spin_max;
    mot_expo = _expo;
    slew_max = _slew_max;
    vehicle_mass = _vehicle_mass;
    diagonal_size = _diagonal_size;
    voltage_max = _voltage_max;
    efficiency = _prop_fom;
    thrust_max = _thrust_max;
    prop_area = _prop_area;

    // assume 50% of mass on ring around center
    moment_of_inertia.x = vehicle_mass * 0.25 * sq(diagonal_size*0.5);
    moment_of_inertia.y = moment_of_inertia.x;
    moment_of_inertia.z = vehicle_mass * 0.5 * sq(diagonal_size*0.5);
}

/*
  convert a PWM value to a command value from 0 to 1
*/
float Motor::pwm_to_command(float pwm) const
{
    const float pwm_thrust_max = mot_pwm_min + mot_spin_max * (mot_pwm_max - mot_pwm_min);
    const float pwm_thrust_min = mot_pwm_min + mot_spin_min * (mot_pwm_max - mot_pwm_min);
    const float pwm_thrust_range = pwm_thrust_max - pwm_thrust_min;
    return constrain_float((pwm-pwm_thrust_min)/pwm_thrust_range, 0, 1);
}

/*
  calculate thrust given a command value
*/
float Motor::calc_thrust(float command, float air_density, float velocity_in) const
{
    // a very approximate model that adjusts hover thrust for air density and airspeed
    float thrust_fraction = sqrtf((1.0f - mot_expo) * command + mot_expo * sq(command));
    const float hover_thrust = thrust_fraction * thrust_max * (air_density / 1.225f);
    const float hover_inflow_velocity = sqrtf(hover_thrust / (2.0f * air_density * prop_area));
    // Assume that for a constant throttle, thrust reduces linearly with airspeed and is zero when
    // airspeed normal to the prop disc is twice the hover inflow value.
    if (hover_inflow_velocity > 0.001f) {
        thrust_fraction *= constrain_float(1.0f - velocity_in / (2.0f * hover_inflow_velocity), 0.0f, 1.1f);
    }
    const float ret = thrust_fraction * thrust_max * (air_density / 1.225f);
#if 0
    static uint32_t counter=0;
    if (command > 0 && counter > 1000) {
        counter=0;
        ::printf("command=%.2f density=%.3f velocity_in=%.2f\n",
                 command, air_density, velocity_in);
        ::printf("%.3f , %.3f , %.3f , %.3f , %.3f\n", hover_thrust, prop_area, hover_inflow_velocity, thrust_max, thrust_fraction);
    }
    counter++;
#endif
    return ret;
}

/*
  calculate motor electrical power required to generate a given thrust
*/
float Motor::calc_power(float thrust, float air_density, float velocity_in) const
{
    // a very approximate model that adjusts hover thrust for air density and airspeed
    float momentum_power;
    if (velocity_in < 0.001f) {
        // use equations for prop in hover
            momentum_power = sqrtf(powf(thrust, 3.0f) / (2.0f * air_density * prop_area));
        } else {
        // solve equations for prop in forward flight
        const float A = 1.0f;
        const float B = 1.0f;
        const float C = - thrust / (2.0f * air_density * prop_area * sq(velocity_in));
        const float a_coef = (-B + sqrtf(sq(B) - 4.0f * A * C)) / (2.0f * A);
        const float momentum_efficiency = 1.0f / (1.0f + a_coef);
        const float thrust_power = thrust * velocity_in;
        momentum_power = thrust_power * momentum_efficiency;
    }
    return momentum_power / efficiency;
}
