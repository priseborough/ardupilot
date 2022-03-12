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
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_Plane.h"
#include <stdio.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

Plane::Plane(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 2.0f;

    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    frame_height = 0.1f;
    num_motors = 1;

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    
    if (strstr(frame_str, "-heavy")) {
        mass = 8;
    }
    if (strstr(frame_str, "-jet")) {
        // a 22kg "jet", level top speed is 102m/s
        mass = 22;
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    }
    if (strstr(frame_str, "-revthrust")) {
        reverse_thrust = true;
    }
    if (strstr(frame_str, "-elevon")) {
        elevons = true;
    } else if (strstr(frame_str, "-vtail")) {
        vtail = true;
    } else if (strstr(frame_str, "-dspoilers")) {
        dspoilers = true;
    }
    // if (strstr(frame_str, "-elevrev")) {
    //     reverse_elevator_rudder = true;
    // }
    // if (strstr(frame_str, "-catapult")) {
        have_launcher = true;
        launch_accel = 15;
        launch_time = 5;
        mass = model.mass;
    // }
    // if (strstr(frame_str, "-bungee")) {
    //     have_launcher = true;
    //     launch_accel = 7;
    //     launch_time = 4;
    // }
    // if (strstr(frame_str, "-throw")) {
    //     have_launcher = true;
    //     launch_accel = 25;
    //     launch_time = 0.4;
    // }
    // if (strstr(frame_str, "-tailsitter")) {
    //     tailsitter = true;
    //     ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
    //     thrust_scale *= 1.5;
    // }

    if (strstr(frame_str, "-ice")) {
        ice_engine = true;
    }

    if (strstr(frame_str, "-soaring")) {
        mass = 2.0;
        coefficient.c_drag_p = 0.05;
    }

    convert_cfd_data(default_cfd_model);

}

// Torque calculation function
Vector3f Plane::getTorque(float inputAileron, float inputElevator, float inputRudder, const Vector3f &force) const
{
    // Calculate dynamic pressure
    const auto &m = model;
    double qPa = 0.5*air_density*sq(velocity_air_bf.length());
    const float aileron_rad = inputAileron * radians(m.aileronDeflectionLimitDeg);
    const float elevator_rad = inputElevator * radians(m.elevatorDeflectionLimitDeg);
    const float rudder_rad = inputRudder * radians(m.rudderDeflectionLimitDeg);
    const float tas = MAX(airspeed * AP::ahrs().get_EAS2TAS(), 1);

    const float delta_alpha = alpharad - m.alphaRef;

    float Cl = (m.Cl2 * sq(delta_alpha) + m.Cl1 * delta_alpha + m.Cl0) * betarad;
    float Cm = m.Cm2 * sq(delta_alpha) + m.Cm1 * delta_alpha + m.Cm0;
    float Cn = (m.Cn2 * sq(delta_alpha) + m.Cn1 * delta_alpha + m.Cn0) * betarad;

    AP::logger().Write("DBG2", "TimeUS,alpha,cm,elev,cm_elev",
                                           "Qffff",
                                           AP_HAL::micros64(),
                                           degrees(alpharad),
                                           Cm,
                                           degrees(elevator_rad),
                                           radians(m.deltaCmperRadianElev));

    Cl += m.deltaClperRadianElev * elevator_rad;
    Cm += m.deltaCmperRadianElev * elevator_rad;
    Cn += m.deltaCnperRadianElev * elevator_rad;

    Cl += m.deltaClperRadianRud * rudder_rad;
    Cm += m.deltaCmperRadianRud * rudder_rad;
    Cn += m.deltaCnperRadianRud * rudder_rad;

    Cl += (m.deltaClperRadianAil2 * sq(delta_alpha) + m.deltaClperRadianAil1 * delta_alpha + m.deltaClperRadianAil0) * aileron_rad;
    Cm += m.deltaCmperRadianAil * aileron_rad;
    Cn += (m.deltaCnperRadianAil2 * sq(delta_alpha) + m.deltaCnperRadianAil1 * delta_alpha + m.deltaCnperRadianAil0) * aileron_rad;

    // derivatives
    float Clp = m.Clp2 * sq(delta_alpha) + m.Clp1 * delta_alpha + m.Clp0;
    float Clr = m.Clr2 * sq(delta_alpha) + m.Clr1 * delta_alpha + m.Clr0;
    float Cnp = m.Cnp2 * sq(delta_alpha) + m.Cnp1 * delta_alpha + m.Cnp0;
    float Cnr = m.Cnr2 * sq(delta_alpha) + m.Cnr1 * delta_alpha + m.Cnr0;

    // normalise gyro rates
    Vector3f pqr_norm = gyro;
    pqr_norm.x *= 0.5 * m.refSpan / tas;
    pqr_norm.y *= 0.5 * m.refChord / tas;
    pqr_norm.z *= 0.5 * m.refSpan / tas;

    Cl += pqr_norm.x * Clp;
    Cl += pqr_norm.z * Clr;
    Cn += pqr_norm.x * Cnp;
    Cn += pqr_norm.z * Cnr;

    Cm += pqr_norm.y * m.Cmq;

    float Mx = Cl * qPa * m.Sref * m.refSpan;
    float My = Cm * qPa * m.Sref * m.refChord;
    float Mz = Cn * qPa * m.Sref * m.refSpan;

#if 0
    AP::logger().Write("MMT2", "TimeUS,alpha,m0,m1,m2,m3,m4,m5,m6,Iyy",
                                           "Qfffffffff",
                                           AP_HAL::micros64(),
                                           degrees(alpharad),
                                           m.Cm0,
                                           m.Cm1 * alpharad,
                                           m.Cm2 * sq(alpharad),
                                           m.deltaCmperRadianElev * elevator_rad,
                                           m.deltaCmperRadianRud * rudder_rad,
                                           m.deltaCmperRadianAil * aileron_rad,
                                           pqr_norm.y * m.Cmq,
                                           m.IYY);
#endif

#if 0
    AP::logger().Write("GLT", "TimeUS,Alpha,Beta,Cl,Cm,Cn", "Qfffff",
                       AP_HAL::micros64(),
                       degrees(alpharad),
                       degrees(betarad),
                       Cl, Cm, Cn);
#endif

    return Vector3f(Mx, My, Mz);
}

// Force calculation, return vector in Newtons
Vector3f Plane::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const auto &m = model;
    const float aileron_rad = inputAileron * radians(m.aileronDeflectionLimitDeg);
    const float elevator_rad = inputElevator * radians(m.elevatorDeflectionLimitDeg);
    const float rudder_rad = inputRudder * radians(m.rudderDeflectionLimitDeg);

    // dynamic pressure
    double qPa = 0.5*air_density*sq(velocity_air_bf.length());

    const float delta_alpha = alpharad - m.alphaRef;

    float CY = (m.CY2 * sq(delta_alpha) + m.CY1 * delta_alpha + m.CY0) * betarad;

    // use alpha sweep data if available
    float CA, CN;
    if (m.n_alpha < 2 || m.n_alpha > 25) {
        // can't do interpolation so use second order fit data
        CA = m.CA2 * sq(delta_alpha) + m.CA1 * delta_alpha + m.CA0;
        CN = m.CN2 * sq(delta_alpha) + m.CN1 * delta_alpha + m.CN0;
    } else {
        // lookup table of CN and CA vs alpha
        static uint8_t last_alpha_index = 0;
        const float alpha_rad_input = constrain_float(alpharad, m.alpha_sweep_rad[0], m.alpha_sweep_rad[m.n_alpha - 1]);
        last_alpha_index = MIN(last_alpha_index, m.n_alpha-2);
        if (alpha_rad_input > m.alpha_sweep_rad[last_alpha_index+1]) {
            if (last_alpha_index < m.n_alpha-2) {
                // search up
                for (uint8_t i=last_alpha_index; i<=m.n_alpha-2; i++) {
                    if (alpha_rad_input <= m.alpha_sweep_rad[last_alpha_index+1] && alpha_rad_input >= m.alpha_sweep_rad[last_alpha_index]) {
                        last_alpha_index = i;
                        break;
                    }
                }
            }
        } else if (alpha_rad_input < m.alpha_sweep_rad[last_alpha_index]) {
            if (last_alpha_index > 0) {
                // search down
                for (int8_t i=last_alpha_index; i>=0; i--) {
                    if (alpha_rad_input <= m.alpha_sweep_rad[last_alpha_index+1] && alpha_rad_input >= m.alpha_sweep_rad[last_alpha_index]) {
                        last_alpha_index = i;
                        break;
                    }
                }
            }
        }
        const float delta_alpha2 = m.alpha_sweep_rad[last_alpha_index+1] - m.alpha_sweep_rad[last_alpha_index];
        const float fraction = (alpha_rad_input - m.alpha_sweep_rad[last_alpha_index]) / delta_alpha2;
        CA = m.CA_sweep[last_alpha_index] + fraction * (m.CA_sweep[last_alpha_index+1] - m.CA_sweep[last_alpha_index]);
        CN = m.CN_sweep[last_alpha_index] + fraction * (m.CN_sweep[last_alpha_index+1] - m.CN_sweep[last_alpha_index]);
    }

#if 1
    const float Cl = CN * cosf(alpharad) - CA * sinf(alphaRad);
    const float Cd = CN * sinf(alphaRad) + CA * cosf(alphaRad);
    AP::logger().Write("DBG1", "TimeUS,alpha,Cl,Cd",
                                           "Qfff",
                                           AP_HAL::micros64(),
                                           degrees(alpharad),
                                           Cl,
                                           Cd);
#endif

    CN += m.deltaCNperRadianElev * elevator_rad;
    CA += m.deltaCAperRadianElev * elevator_rad;
    CY += m.deltaCYperRadianElev * elevator_rad;

    CN += m.deltaCNperRadianRud * rudder_rad;
    CA += m.deltaCAperRadianRud * rudder_rad;
    CY += m.deltaCYperRadianRud * rudder_rad;

    CN += m.deltaCNperRadianAil * aileron_rad;
    CA += m.deltaCAperRadianAil * aileron_rad;
    CY += m.deltaCYperRadianAil * aileron_rad;
    
    float Fx = -CA * qPa * m.Sref;
    float Fy =  CY * qPa * m.Sref;
    float Fz = -CN * qPa * m.Sref;

    Vector3f ret = Vector3f(Fx, Fy, Fz);

    return ret;
}

void Plane::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    // mapping is:
    // RC1 -> front right
    // RC2 -> rear left
    // RC3 -> front left
    // RC4 -> rear right
    // Assume positive is te down.
    // TBD better handle scaling and limits when more data is provided

    // these deflections are nromalised between +-1
    const float servo_front_right_defln = filtered_servo_angle(input, 0);
    const float servo_rear_left_defln   = filtered_servo_angle(input, 1);
    const float servo_front_left_defln  = filtered_servo_angle(input, 2);
    const float servo_rear_right_defln  = filtered_servo_angle(input, 3);

    float aileron  = 0.25 * ( - servo_front_right_defln + servo_rear_left_defln + servo_front_left_defln - servo_rear_right_defln);
    float elevator = 0.25 * ( + servo_front_right_defln - servo_rear_left_defln + servo_front_left_defln - servo_rear_right_defln);
    float rudder   = 0.0f;

#if 0
    AP::logger().Write("MMT1", "TimeUS,pwm1,defln1,pwm2,defln2,pwm3,defln3,pwm4,defln4",
                                           "QIfIfIfIf",
                                           AP_HAL::micros64(),
                                           input.servos[0],
                                           servo_front_right_defln,
                                           input.servos[1],
                                           servo_rear_left_defln,
                                           input.servos[2],
                                           servo_front_left_defln,
                                           input.servos[3],
                                           servo_rear_right_defln);
#endif

    // calculate angle of attack
    alpharad = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    betarad = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    alpharad = constrain_float(alpharad, -model.alphaRadMax, model.alphaRadMax);
    betarad = constrain_float(betarad, -model.betaRadMax, model.betaRadMax);

    // update airspeed and density
    const float air_density_ratio = AP::baro().get_air_density_ratio();
    air_density = SSL_AIR_DENSITY * air_density_ratio;

    Vector3f force = getForce(aileron, elevator, rudder);
    Vector3f moment = getTorque(aileron, elevator, rudder, force);

    // correct moments for aerodynamic moment reference centre offset from c.g.
    const Vector3f offset = Vector3f( - model.Xcg, - model.Ycg, - model.Zcg);
    moment += offset % force;

    // calculate angular accelerations ingonring cross products of inertia
    // TODO include cross-products
    rot_accel = Vector3f(moment.x/model.IXX, moment.y/model.IYY, moment.z/model.IZZ);

    if (have_launcher) {
        bool release_triggered = input.servos[6] > 1700;
        if (release_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
            }
        } else {
            launch_start_ms = 0;
        }
    }

    accel_body = force / model.mass;

    if (on_ground()) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }

    // constrain accelerations
    accel_body.x = constrain_float(accel_body.x, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
    accel_body.y = constrain_float(accel_body.y, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
    accel_body.z = constrain_float(accel_body.z, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
}

/*
  update the plane simulation by one time step
 */
void Plane::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

#if 0
    static bool done_first;
    if (!done_first) {
        done_first = true;
        position.z = -1500;
        velocity_ef.x = -50;
    }
#endif

    update_wind(input);
    
    calculate_forces(input, rot_accel, accel_body);
    
    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

void Plane::convert_cfd_data(ModelCFD &cfd) {
    if (cfd.n_alpha >= 2 && cfd.n_alpha <= 25) {
        model.n_alpha = cfd.n_alpha;
        for (uint8_t i=0; i<cfd.n_alpha; i++) {
            model.alpha_sweep_rad[i] = radians(cfd.alpha_sweep_deg[i]);
            model.CA_sweep[i] = cfd.CFx_sweep[i];
            model.CN_sweep[i] = cfd.CFz_sweep[i];
        }
    }

    // calculate AoA and AoS force and moment derivatives
    model.alphaRef = radians(cfd.AoA_ref);
    const float delta_alpha_inv = 1.0f / cfd.delta_alpha;
    const float delta_beta_inv = 1.0f / cfd.delta_beta;

    model.CA0 = cfd.Base_Aero[CFx];
    model.CA1 = (cfd.Alpha_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_alpha_inv;
    model.CA2 = 0;

    model.CY0 = (cfd.Beta_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_beta_inv;
    model.CY1 = 0;
    model.CY2 = 0;

    model.CN0 = cfd.Base_Aero[CFz];
    model.CN1 = (cfd.Alpha_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_alpha_inv;
    model.CN2 = 0;

    model.Cl0 = - (cfd.Beta_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_beta_inv;
    model.Cl1 = 0;
    model.Cl2 = 0;

    model.Cm0 = cfd.Base_Aero[CMy];
    model.Cm1 = (cfd.Alpha_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_alpha_inv;
    model.Cm2 = 0;

    model.Cn0 = - (cfd.Beta_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_beta_inv;
    model.Cn1 = 0;
    model.Cn2 = 0;

    // calculate dynamic derivatives
    Vector3f pqr_norm = Vector3f(cfd.delta_roll_rate, cfd.delta_pitch_rate, cfd.delta_yaw_rate);
    pqr_norm.x *= 0.5f * cfd.Bref / cfd.Vinf;
    pqr_norm.y *= 0.5f * cfd.Cref / cfd.Vinf;
    pqr_norm.z *= 0.5f * cfd.Bref / cfd.Vinf;

    model.Clp0 = - (cfd.Roll_Rate_Delta[CMx] - cfd.Base_Aero[CMx]) / pqr_norm.x;
    model.Clp1 = 0;
    model.Clp2 = 0;

    model.Cnp0 = - (cfd.Roll_Rate_Delta[CMz] - cfd.Base_Aero[CMz]) / pqr_norm.x;
    model.Cnp1 = 0;
    model.Cnp2 = 0;

    model.Clr0 = - (cfd.Yaw_Rate_Delta[CMx] - cfd.Base_Aero[CMx]) / pqr_norm.z;
    model.Clr1 = 0;
    model.Clr2 = 0;

    model.Cnr0 = - (cfd.Yaw_Rate_Delta[CMz] - cfd.Base_Aero[CMz]) / pqr_norm.z;
    model.Cnr1 = 0;
    model.Cnr2 = 0;

    model.Cmq = (cfd.Pitch_Rate_Delta[CMz] - cfd.Base_Aero[CMz]) / pqr_norm.y;

    // control surface derivatives
    const float delta_inv  = 1.0f / cfd.delta_control;

    // aileron
    model.aileronDeflectionLimitDeg = cfd.aileronDeflectionLimitDeg;

    // front right control surface - positive is down
    model.deltaCAperRadianAil  =   cfd.ail_to_fr * (cfd.Front_Right_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_inv;
    model.deltaCYperRadianAil  =   cfd.ail_to_fr * (cfd.Front_Right_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_inv;
    model.deltaCNperRadianAil  =   cfd.ail_to_fr * (cfd.Front_Right_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_inv;
    model.deltaClperRadianAil0 = - cfd.ail_to_fr * (cfd.Front_Right_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_inv;
    model.deltaCmperRadianAil  =   cfd.ail_to_fr * (cfd.Front_Right_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_inv;
    model.deltaCnperRadianAil0 = - cfd.ail_to_fr * (cfd.Front_Right_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_inv;

    // front left control surface - positive is up
    model.deltaCAperRadianAil  +=   cfd.ail_to_fl * (cfd.Front_Left_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_inv;
    model.deltaCYperRadianAil  +=   cfd.ail_to_fl * (cfd.Front_Left_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_inv;
    model.deltaCNperRadianAil  +=   cfd.ail_to_fl * (cfd.Front_Left_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_inv;
    model.deltaClperRadianAil0 += - cfd.ail_to_fl * (cfd.Front_Left_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_inv;
    model.deltaCmperRadianAil  +=   cfd.ail_to_fl * (cfd.Front_Left_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_inv;
    model.deltaCnperRadianAil0 += - cfd.ail_to_fl * (cfd.Front_Left_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_inv;

    // rear right control surface - positive is down
    model.deltaCAperRadianAil  +=   cfd.ail_to_rr * (cfd.Rear_Right_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_inv;
    model.deltaCYperRadianAil  +=   cfd.ail_to_rr * (cfd.Rear_Right_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_inv;
    model.deltaCNperRadianAil  +=   cfd.ail_to_rr * (cfd.Rear_Right_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_inv;
    model.deltaClperRadianAil0 += - cfd.ail_to_rr * (cfd.Rear_Right_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_inv;
    model.deltaCmperRadianAil  +=   cfd.ail_to_rr * (cfd.Rear_Right_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_inv;
    model.deltaCnperRadianAil0 += - cfd.ail_to_rr * (cfd.Rear_Right_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_inv;

    // rear left control surface - positive is up
    model.deltaCAperRadianAil  +=   cfd.ail_to_rl * (cfd.Rear_Left_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_inv;
    model.deltaCYperRadianAil  +=   cfd.ail_to_rl * (cfd.Rear_Left_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_inv;
    model.deltaCNperRadianAil  +=   cfd.ail_to_rl * (cfd.Rear_Left_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_inv;
    model.deltaClperRadianAil0 += - cfd.ail_to_rl * (cfd.Rear_Left_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_inv;
    model.deltaCmperRadianAil  +=   cfd.ail_to_rl * (cfd.Rear_Left_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_inv;
    model.deltaCnperRadianAil0 += - cfd.ail_to_rl * (cfd.Rear_Left_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_inv;

    // no data for higher order terms
    model.deltaClperRadianAil1 = 0;
    model.deltaClperRadianAil2 = 0;
    model.deltaCnperRadianAil1 = 0;
    model.deltaCnperRadianAil2 = 0;

    // elevator
    model.elevatorDeflectionLimitDeg = cfd.elevatorDeflectionLimitDeg;

    // front right control surface
    model.deltaCAperRadianElev =   cfd.ele_to_fr * (cfd.Front_Right_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_inv;
    model.deltaCYperRadianElev =   cfd.ele_to_fr * (cfd.Front_Right_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_inv;
    model.deltaCNperRadianElev =   cfd.ele_to_fr * (cfd.Front_Right_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_inv;
    model.deltaClperRadianElev = - cfd.ele_to_fr * (cfd.Front_Right_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_inv;
    model.deltaCmperRadianElev =   cfd.ele_to_fr * (cfd.Front_Right_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_inv;
    model.deltaCnperRadianElev = - cfd.ele_to_fr * (cfd.Front_Right_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_inv;

    // front left control surface
    model.deltaCAperRadianElev +=   cfd.ele_to_fl * (cfd.Front_Left_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_inv;
    model.deltaCYperRadianElev +=   cfd.ele_to_fl * (cfd.Front_Left_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_inv;
    model.deltaCNperRadianElev +=   cfd.ele_to_fl * (cfd.Front_Left_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_inv;
    model.deltaClperRadianElev += - cfd.ele_to_fl * (cfd.Front_Left_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_inv;
    model.deltaCmperRadianElev +=   cfd.ele_to_fl * (cfd.Front_Left_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_inv;
    model.deltaCnperRadianElev += - cfd.ele_to_fl * (cfd.Front_Left_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_inv;

    // rear right control surface
    model.deltaCAperRadianElev +=   cfd.ele_to_rr * (cfd.Rear_Right_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_inv;
    model.deltaCYperRadianElev +=   cfd.ele_to_rr * (cfd.Rear_Right_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_inv;
    model.deltaCNperRadianElev +=   cfd.ele_to_rr * (cfd.Rear_Right_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_inv;
    model.deltaClperRadianElev += - cfd.ele_to_rr * (cfd.Rear_Right_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_inv;
    model.deltaCmperRadianElev +=   cfd.ele_to_rr * (cfd.Rear_Right_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_inv;
    model.deltaCnperRadianElev += - cfd.ele_to_rr * (cfd.Rear_Right_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_inv;

    // rear left control surface
    model.deltaCAperRadianElev +=   cfd.ele_to_rl * (cfd.Rear_Left_Delta[CFx] - cfd.Base_Aero[CFx]) * delta_inv;
    model.deltaCYperRadianElev +=   cfd.ele_to_rl * (cfd.Rear_Left_Delta[CFy] - cfd.Base_Aero[CFy]) * delta_inv;
    model.deltaCNperRadianElev +=   cfd.ele_to_rl * (cfd.Rear_Left_Delta[CFz] - cfd.Base_Aero[CFz]) * delta_inv;
    model.deltaClperRadianElev += - cfd.ele_to_rl * (cfd.Rear_Left_Delta[CMx] - cfd.Base_Aero[CMx]) * delta_inv;
    model.deltaCmperRadianElev +=   cfd.ele_to_rl * (cfd.Rear_Left_Delta[CMy] - cfd.Base_Aero[CMy]) * delta_inv;
    model.deltaCnperRadianElev += - cfd.ele_to_rl * (cfd.Rear_Left_Delta[CMz] - cfd.Base_Aero[CMz]) * delta_inv;

    // rudder
    model.rudderDeflectionLimitDeg = 0.0f;
    model.deltaCNperRadianRud = 0.0f;
    model.deltaCAperRadianRud = 0.0f;
    model.deltaCYperRadianRud = 0.0f;
    model.deltaClperRadianRud = 0.0f;
    model.deltaCmperRadianRud = 0.0f;
    model.deltaCnperRadianRud = 0.0f;

    // AoA and AoS limits
    model.alphaRadMax = radians(cfd.alphaMaxDeg);
    model.betaRadMax = radians(cfd.betaMaxDeg);

    // position of c.g. wrt the aero moment reference centre
    model.Xcg = cfd.Xref - cfd.Xcg; // CFD X axis points rearwards
    model.Ycg = cfd.Ycg - cfd.Yref; // CFD Y axis points right
    model.Zcg = cfd.Zref - cfd.Zcg; // CFD Z axis points up

    model.mass = cfd.mass;
    model.IXX = cfd.IXX;
    model.IYY = cfd.IYY;
    model.IZZ = cfd.IZZ;

}
