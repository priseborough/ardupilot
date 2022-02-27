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

#include "SIM_PlaneJSON.h"

#include <stdio.h>
#include "picojson.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

PlaneJSON::PlaneJSON(const char *frame_str) :
    Aircraft(frame_str)
{
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    carriage_state = carriageState::WAITING_FOR_PICKUP;
    if (USE_CFD_DATA) {
        model = convert_cfd_data(default_cfd_model);
    } else {
        model = default_model;
    }
    auto *_sitl = AP::sitl();
    _sitl->setalt.set_and_save(0);
    _sitl->setspeed.set_and_save(0);
    _sitl->setpitch.set_and_save(0);
}

Model PlaneJSON::convert_cfd_data(ModelCFD &cfd_model) {
    Model model = default_model;

    // calculate AoA and AoS force and moment derivatives
    const float delta_alpha_inv = 1.0f / cfd_model.delta_alpha;
    const float delta_beta_inv = 1.0f / cfd_model.delta_beta;

    model.CA0 = cfd_model.Base_Aero[CFx];
    model.CA1 = (cfd_model.Alpha_Delta[CFx] - cfd_model.Base_Aero[CFx]) * delta_alpha_inv;
    model.CA2 = 0;

    model.CY0 = (cfd_model.Beta_Delta[CFy] - cfd_model.Base_Aero[CFy]) * delta_beta_inv;
    model.CY1 = 0;
    model.CY2 = 0;

    model.CN0 = cfd_model.Base_Aero[CFz];
    model.CN1 = (cfd_model.Alpha_Delta[CFz] - cfd_model.Base_Aero[CFz]) * delta_alpha_inv;
    model.CN2 = 0;

    model.Cl0 = - (cfd_model.Beta_Delta[CMx] - cfd_model.Base_Aero[CMx]) * delta_beta_inv;
    model.Cl1 = 0;
    model.Cl2 = 0;

    model.Cm0 = cfd_model.Base_Aero[CMy];
    model.Cm1 = (cfd_model.Alpha_Delta[CMy] - cfd_model.Base_Aero[CMy]) * delta_alpha_inv;
    model.Cm2 = 0;

    model.Cn0 = - (cfd_model.Beta_Delta[CMz] - cfd_model.Base_Aero[CMz]) * delta_beta_inv;
    model.Cn1 = 0;
    model.Cn2 = 0;

    // calculate dynamic derivatives
    Vector3f pqr_norm = Vector3f(cfd_model.delta_roll_rate, cfd_model.delta_pitch_rate, cfd_model.delta_yaw_rate);
    pqr_norm.x *= 0.5f * cfd_model.Bref / cfd_model.Vinf;
    pqr_norm.y *= 0.5f * cfd_model.Cref / cfd_model.Vinf;
    pqr_norm.z *= 0.5f * cfd_model.Bref / cfd_model.Vinf;

    model.Clp0 = - (cfd_model.Roll_Rate_Delta[CMx] - cfd_model.Base_Aero[CMx]) / pqr_norm.x;
    model.Clp1 = 0;
    model.Clp2 = 0;

    model.Cnp0 = - (cfd_model.Roll_Rate_Delta[CMz] - cfd_model.Base_Aero[CMz]) / pqr_norm.x;
    model.Cnp1 = 0;
    model.Cnp2 = 0;

    model.Clr0 = - (cfd_model.Yaw_Rate_Delta[CMx] - cfd_model.Base_Aero[CMx]) / pqr_norm.z;
    model.Clr1 = 0;
    model.Clr2 = 0;

    model.Cnr0 = - (cfd_model.Yaw_Rate_Delta[CMz] - cfd_model.Base_Aero[CMz]) / pqr_norm.z;
    model.Cnr1 = 0;
    model.Cnr2 = 0;

    model.Cmq = (cfd_model.Pitch_Rate_Delta[CMz] - cfd_model.Base_Aero[CMz]) / pqr_norm.y;

    // calculate control derivatives - TBD values

    // aileron
    model.aileronDeflectionLimitDeg = cfd_model.aileronDeflectionLimitDeg;
    const float delta_ail_inv = 1.0f / cfd_model.delta_aileron;
    model.deltaClperRadianAil0 = - (cfd_model.Aileron_Delta[CMx] - cfd_model.Base_Aero[CMx]) * delta_ail_inv;
    model.deltaClperRadianAil1 = 0;
    model.deltaClperRadianAil2 = 0;
    model.deltaCnperRadianAil0 = - (cfd_model.Aileron_Delta[CMz] - cfd_model.Base_Aero[CMz]) * delta_ail_inv;
    model.deltaCnperRadianAil1 = 0;
    model.deltaCnperRadianAil2 = 0;
    model.deltaCNperRadianAil = (cfd_model.Aileron_Delta[CFz] - cfd_model.Base_Aero[CFz]) * delta_ail_inv;
    model.deltaCAperRadianAil = (cfd_model.Aileron_Delta[CFx] - cfd_model.Base_Aero[CFx]) * delta_ail_inv;
    model.deltaCmperRadianAil = (cfd_model.Aileron_Delta[CMy] - cfd_model.Base_Aero[CMy]) * delta_ail_inv;
    model.deltaCYperRadianAil = (cfd_model.Aileron_Delta[CFy] - cfd_model.Base_Aero[CFy]) * delta_ail_inv;   

    // elevator
    model.elevatorDeflectionLimitDeg = cfd_model.elevatorDeflectionLimitDeg;
    const float delta_elev_inv = 1.0f / cfd_model.delta_elevator;
    model.deltaCNperRadianElev = 0;
    model.deltaCAperRadianElev = (cfd_model.Elevator_Delta[CFx] - cfd_model.Base_Aero[CFx]) * delta_elev_inv;
    model.deltaCmperRadianElev = (cfd_model.Elevator_Delta[CMy] - cfd_model.Base_Aero[CMy]) * delta_elev_inv;
    model.deltaCYperRadianElev = 0;
    model.deltaClperRadianElev = 0;
    model.deltaCnperRadianElev = 0;

    // rudder
    model.rudderDeflectionLimitDeg = cfd_model.rudderDeflectionLimitDeg;
    const float delta_rudd_inv = cfd_model.delta_rudder;
    model.deltaCNperRadianRud = 0;
    model.deltaCAperRadianRud = 0.058;
    model.deltaCmperRadianRud = 0;
    model.deltaCYperRadianRud = 0.31;
    model.deltaClperRadianRud = 0.038;
    model.deltaCnperRadianRud = -0.174;

}

// Torque calculation function
Vector3f PlaneJSON::getTorque(float inputAileron, float inputElevator, float inputRudder, const Vector3f &force) const
{
    // Calculate dynamic pressure
    const auto &m = model;
    double qPa = 0.5*air_density*sq(velocity_air_bf.length());
    const float aileron_rad = inputAileron * radians(m.aileronDeflectionLimitDeg);
    const float elevator_rad = inputElevator * radians(m.elevatorDeflectionLimitDeg);
    const float rudder_rad = inputRudder * radians(m.rudderDeflectionLimitDeg);
    const float tas = MAX(airspeed * AP::ahrs().get_EAS2TAS(), 1);

    float Cl = (m.Cl2 * sq(alpharad) + m.Cl1 * alpharad + m.Cl0) * betarad;
    float Cm = m.Cm2 * sq(alpharad) + m.Cm1 * alpharad + m.Cm0;
    float Cn = (m.Cn2 * sq(alpharad) + m.Cn1 * alpharad + m.Cn0) * betarad;

    Cl += m.deltaClperRadianElev * elevator_rad;
    Cm += m.deltaCmperRadianElev * elevator_rad;
    Cn += m.deltaCnperRadianElev * elevator_rad;

    Cl += m.deltaClperRadianRud * rudder_rad;
    Cm += m.deltaCmperRadianRud * rudder_rad;
    Cn += m.deltaCnperRadianRud * rudder_rad;

    Cl += (m.deltaClperRadianAil2 * sq(alpharad) + m.deltaClperRadianAil1 * alpharad + m.deltaClperRadianAil0) * aileron_rad;
    Cm += m.deltaCmperRadianAil * aileron_rad;
    Cn += (m.deltaCnperRadianAil2 * sq(alpharad) + m.deltaCnperRadianAil1 * alpharad + m.deltaCnperRadianAil0) * aileron_rad;

    // derivatives
    float Clp = m.Clp2 * sq(alpharad) + m.Clp1 * alpharad + m.Clp0;
    float Clr = m.Clr2 * sq(alpharad) + m.Clr1 * alpharad + m.Clr0;
    float Cnp = m.Cnp2 * sq(alpharad) + m.Cnp1 * alpharad + m.Cnp0;
    float Cnr = m.Cnr2 * sq(alpharad) + m.Cnr1 * alpharad + m.Cnr0;

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
    AP::logger().Write("GLT", "TimeUS,Alpha,Beta,Cl,Cm,Cn", "Qfffff",
                       AP_HAL::micros64(),
                       degrees(alpharad),
                       degrees(betarad),
                       Cl, Cm, Cn);
#endif

    return Vector3f(Mx/m.IXX, My/m.IYY, Mz/m.IZZ);
}

// Force calculation, return vector in Newtons
Vector3f PlaneJSON::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const auto &m = model;
    const float aileron_rad = inputAileron * radians(m.aileronDeflectionLimitDeg);
    const float elevator_rad = inputElevator * radians(m.elevatorDeflectionLimitDeg);
    const float rudder_rad = inputRudder * radians(m.rudderDeflectionLimitDeg);

    // dynamic pressure
    double qPa = 0.5*air_density*sq(velocity_air_bf.length());

    float CA = m.CA2 * sq(alpharad) + m.CA1 * alpharad + m.CA0;
    float CY = (m.CY2 * sq(alpharad) + m.CY1 * alpharad + m.CY0) * betarad;
    float CN = m.CN2 * sq(alpharad) + m.CN1 * alpharad + m.CN0;

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

    float Flift = Fx * sin(alpharad)  - Fz * cos(alpharad);
    float Fdrag = -Fx * cos(alpharad) - Fz * sin(alpharad);

    if (carriage_state == carriageState::RELEASED) {
        static uint32_t last_drag_ms;
        static float sim_LD;
        uint32_t now = AP_HAL::millis();
        sim_LD = 0.1 * constrain_float(Flift/MAX(1.0e-6,Fdrag),0,20) + 0.9 * sim_LD;
        if (now - last_drag_ms > 10 &&
            airspeed > 1) {
            last_drag_ms = now;
            AP::logger().Write("SLD", "TimeUS,AltFt,AltM,EAS,TAS,AD,Fl,Fd,LD,Elev,AoA,Fx,Fy,Fz,q", "Qffffffffffffff",
                               AP_HAL::micros64(),
                               (location.alt*0.01)/FEET_TO_METERS,
                               location.alt*0.01,
                               velocity_air_bf.length()/eas2tas,
                               velocity_air_bf.length(),
                               air_density,
                               Flift, Fdrag, sim_LD,
                               degrees(elevator_rad),
                               degrees(alpharad),
                               Fx, Fy, Fz,
                               qPa);
            AP::logger().Write("SL2", "TimeUS,AltFt,KEAS,KTAS,AD,Fl,Fd,LD,Elev,Ail,Rud,AoA,SSA,q,Az", "Qffffffffffffff",
                               AP_HAL::micros64(),
                               (location.alt*0.01)/FEET_TO_METERS,
                               M_PER_SEC_TO_KNOTS*velocity_air_bf.length()/eas2tas,
                               M_PER_SEC_TO_KNOTS*velocity_air_bf.length(),
                               air_density,
                               Flift, Fdrag, sim_LD,
                               degrees(elevator_rad),
                               degrees(aileron_rad),
                               degrees(rudder_rad),
                               degrees(alpharad),
                               degrees(betarad),
                               qPa,
                               accel_body.z);

            AP::logger().Write("SCTL", "TimeUS,Ail,Elev,Rudd", "Qfff",
                               AP_HAL::micros64(),
                               degrees(aileron_rad),
                               degrees(elevator_rad),
                               degrees(rudder_rad));
        }
    }


    return ret;
}

void PlaneJSON::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    auto *_sitl = AP::sitl();

    filtered_servo_setup(1, 1100, 1900, model.aileronDeflectionLimitDeg);
    filtered_servo_setup(4, 1100, 1900, model.aileronDeflectionLimitDeg);
    filtered_servo_setup(2, 1100, 1900, model.elevatorDeflectionLimitDeg);
    filtered_servo_setup(3, 1100, 1900, model.rudderDeflectionLimitDeg);
    
    float aileron  = 0.5*(filtered_servo_angle(input, 1, 400) + filtered_servo_angle(input, 4, 400));
    float elevator = filtered_servo_angle(input, 2, 400);
    float rudder   = filtered_servo_angle(input, 3, 400);
    float throttle = 0;
    float balloon  = filtered_servo_range(input, 5);
    float balloon_cut = filtered_servo_range(input, 9);

    // Move balloon upwards using balloon velocity from channel 6
    // Aircraft is released from ground constraint when channel 6 PWM > 1010
    // Once released, plane will be dropped when balloon_burst_amsl is reached or channel 6 is set to PWM 1000
    if (carriage_state == carriageState::WAITING_FOR_RELEASE) {
        balloon_velocity = Vector3f(-wind_ef.x, -wind_ef.y, -wind_ef.z -_sitl->balloon_rate * balloon);
        balloon_position += balloon_velocity * (1.0e-6f * (float)frame_time_us);
        const float height_AMSL = 0.01f * (float)home.alt - position.z;
        // release at burst height or when channel 9 goes high
        if (hal.scheduler->is_system_initialized() &&
            (height_AMSL > _sitl->balloon_burst_amsl || balloon_cut > 0.8)) {
            ::printf("pre-release at %i m AMSL\n", (int)height_AMSL);
            carriage_state = carriageState::PRE_RELEASE;
        }
    } else if (carriage_state == carriageState::PRE_RELEASE) {
        // slow down for release
        balloon_velocity *= 0.999;
        balloon_position += balloon_velocity * (1.0e-6f * (float)frame_time_us);
        if (balloon_velocity.length() < 0.5) {
            carriage_state = carriageState::RELEASED;
            use_smoothing = false;
            const float height_AMSL = 0.01f * (float)home.alt - position.z;
            ::printf("released at %i m AMSL\n", (int)height_AMSL);
        }
    } else if (carriage_state == carriageState::WAITING_FOR_PICKUP) {
        // Don't allow the balloon to drag sideways until the pickup
        balloon_velocity = Vector3f(0.0f, 0.0f, -_sitl->balloon_rate * balloon);
        balloon_position += balloon_velocity * (1.0e-6f * (float)frame_time_us);
    }

    // calculate angle of attack
    alpharad = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    betarad = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    alpharad = constrain_float(alpharad, -model.alphaRadMax, model.alphaRadMax);
    betarad = constrain_float(betarad, -model.betaRadMax, model.betaRadMax);

    Vector3f force;

    if (!update_balloon(balloon, force, rot_accel)) {
        force = getForce(aileron, elevator, rudder);
        rot_accel = getTorque(aileron, elevator, rudder, force);

        // scale thrust to match nose up hover throttle
        float thrust_scale = (model.mass * GRAVITY_MSS) / model.hoverThrottle;
        float thrust   = throttle * thrust_scale;
        force += Vector3f(thrust, 0, 0);
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

    if (hal.util->get_soft_armed()) {
        if (!is_zero(_sitl->setalt)) {
            float delta = _sitl->setalt + position.z;
            position.z -= delta;
            balloon_position.z -= delta;
            ::printf("setalt to %.2fm\n", _sitl->setalt.get());
            _sitl->setalt.set_and_save(0);
        }
        if (!is_zero(_sitl->setpitch)) {
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            dcm.from_euler(r, radians(_sitl->setpitch), y);
        }
        if (!on_ground() && !is_zero(_sitl->setspeed)) {
            float eas = _sitl->setspeed.get();
            float tas = eas * eas2tas;
            float tas_old = velocity_air_bf.x;
            float scale = is_zero(tas_old)?1:tas / tas_old;
            ::printf("speed scale=%.9f\n", scale);
            velocity_air_bf *= scale;
            velocity_air_ef *= scale;
            velocity_ef *= scale;
        }
    }
}
    
/*
  update the plane simulation by one time step
 */
void PlaneJSON::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);
    
    calculate_forces(input, rot_accel, accel_body);
    
    if (carriage_state == carriageState::WAITING_FOR_PICKUP) {
        // Handle special case where plane is being held nose down waiting to be lifted
        accel_body = dcm.transposed() * Vector3f(0.0f, 0.0f, -GRAVITY_MSS);
        velocity_ef.zero();
        gyro.zero();
        dcm.from_euler(0.0f, radians(-80.0f), radians(home_yaw));
        use_smoothing = true;
        adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));
    } else {
        update_dynamics(rot_accel);
    }
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

/*
   return true if we are on the ground
*/
bool PlaneJSON::on_ground() const
{
    // prevent bouncing around ground
    // prevent bouncing around ground
    // don't do ground interaction if being carried
    return (hagl() <= 0.001f &&
            carriage_state != carriageState::WAITING_FOR_PICKUP &&
            carriage_state != carriageState::PRE_RELEASE &&
            carriage_state != carriageState::WAITING_FOR_RELEASE);
}

/*
  implement balloon lift
  controlled by SIM_BLN_BURST and SIM_BLN_RATE
 */
bool PlaneJSON::update_balloon(float balloon, Vector3f &force, Vector3f &rot_accel)
{
    if (!hal.util->get_soft_armed()) {
        return false;
    }

    if (carriage_state != carriageState::WAITING_FOR_PICKUP &&
        carriage_state != carriageState::PRE_RELEASE &&
        carriage_state != carriageState::WAITING_FOR_RELEASE) {
        // balloon not active
        return false;
    }

    // assume a 50m tether with a 1Hz pogo frequency and damping ratio of 0.2
    Vector3f tether_pos_bf = Vector3f(-1.0f,0.0f,0.0f); // tether attaches to vehicle tail approx 1m behind c.g.
    const float omega = model.tetherPogoFreq * M_2PI; // rad/sec
    const  float zeta = 0.7f;
    float tether_stiffness = model.mass * sq(omega); // N/m
    float tether_damping = 2.0f * zeta * omega / model.mass; // N/(m/s)
    // NED relative position vector from tether attachment on plane to balloon attachment
    Vector3d relative_position = balloon_position.todouble() - (position + (dcm * tether_pos_bf).todouble());
    const float separation_distance = relative_position.length();

    // NED unit vector pointing from tether attachment on plane to attachment on balloon
    Vector3f tether_unit_vec_ef = relative_position.normalized().tofloat();

    // NED velocity of attahment point on plane
    Vector3f attachment_velocity_ef = velocity_ef + dcm * (gyro % tether_pos_bf);

    // NED velocity of attachment point on balloon as seen by observer on attachemnt point on plane
    Vector3f relative_velocity = balloon_velocity - attachment_velocity_ef;

    float separation_speed = relative_velocity * tether_unit_vec_ef;

    // rate increase in separation between attachment point on plane and balloon
    // tension force in tether due to stiffness and damping
    float tension_force = MAX(0.0f, (separation_distance - model.tetherLength) * tether_stiffness);
    if (tension_force > 0.0f) {
        tension_force += constrain_float(separation_speed * tether_damping, 0.0f, 0.05f * tension_force);
    }

    if (carriage_state == carriageState::WAITING_FOR_PICKUP && tension_force > 1.2f * model.mass * GRAVITY_MSS && balloon > 0.01f) {
        carriage_state = carriageState::WAITING_FOR_RELEASE;
    }

    if (carriage_state == carriageState::WAITING_FOR_RELEASE ||
        carriage_state == carriageState::PRE_RELEASE) {
        Vector3f tension_force_vector_ef = tether_unit_vec_ef * tension_force;
        Vector3f tension_force_vector_bf = dcm.transposed() * tension_force_vector_ef;
        force = tension_force_vector_bf;

        // drag force due to lateral motion assuming projected area from Y is 20% of projected area seen from Z and
        // assuming bluff body drag characteristic. In reality we would need an aero model that worked flying backwards,
        // but this will have to do for now.
        Vector3f aero_force_bf = Vector3f(0.0f, 0.2f * velocity_air_bf.y * fabsf(velocity_air_bf.y), velocity_air_bf.z * fabsf(velocity_air_bf.z));
        aero_force_bf *= air_density * model.Sref;
        force -= aero_force_bf;

        Vector3f tension_moment_vector_bf = tether_pos_bf % tension_force_vector_bf;
        Vector3f tension_rot_accel = Vector3f(tension_moment_vector_bf.x/model.IXX, tension_moment_vector_bf.y/model.IYY, tension_moment_vector_bf.z/model.IZZ);
        rot_accel = tension_rot_accel;

        // add some rotation damping due to air resistance assuming a 2 sec damping time constant at SL density
        // TODO model roll damping with more accuracy using Clp data for zero alpha as a first approximation
        rot_accel -= gyro * 0.5f * air_density;
    } else {
        // thether is either slack awaiting pickup or released
        rot_accel.zero();
        force = dcm.transposed() * Vector3f(0.0f, 0.0f, -GRAVITY_MSS * model.mass);
    }

    // balloon is active
    return true;
}
