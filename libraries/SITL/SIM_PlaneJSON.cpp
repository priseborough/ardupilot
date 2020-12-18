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

using namespace SITL;

PlaneJSON::PlaneJSON(const char *frame_str) :
    Aircraft(frame_str)
{
    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    model = default_model;
}

// Torque calculation function
Vector3f PlaneJSON::getTorque(float inputAileron, float inputElevator, float inputRudder, const Vector3f &force) const
{
    // Calculate dynamic pressure
    const auto &m = model;
    float rho = air_density;
    double qPa = 0.5*rho*sq(airspeed);
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


    AP::logger().Write("GLT", "TimeUS,Alpha,Beta,Cl,Cm,Cn", "Qfffff",
                       AP_HAL::micros64(),
                       degrees(alpharad),
                       degrees(betarad),
                       Cl, Cm, Cn);

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
    float rho = air_density;
    double qPa = 0.5*rho*sq(airspeed);

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

    AP::logger().Write("GLF", "TimeUS,Alpha,Beta,CA,CY,CN", "Qfffff",
                       AP_HAL::micros64(),
                       degrees(alpharad),
                       degrees(betarad),
                       CA, CY, CN);
    
    return Vector3f(Fx, Fy, Fz);
}

void PlaneJSON::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float rudder   = filtered_servo_angle(input, 3);
    float throttle = filtered_servo_range(input, 2);
    float balloon  = filtered_servo_range(input, 5);

    // calculate angle of attack
    alpharad = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    betarad = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    alpharad = constrain_float(alpharad, -model.alphaRadMax, model.alphaRadMax);
    betarad = constrain_float(betarad, -model.betaRadMax, model.betaRadMax);

    Vector3f force = getForce(aileron, elevator, rudder);
    rot_accel = getTorque(aileron, elevator, rudder, force);

    // add forces and moments due to balloon tether

    // move balloon upwards using balloon velocity from channel 6
    const float ballon_release_hgt = 1000.0f;
    if (balloon > 0.01 && !balloon_released) {
       const float balloon_climb_rate = 10.0f;
       balloon_velocity = Vector3f(wind_ef.x, wind_ef.y, -balloon_climb_rate * balloon);
       if (balloon_position.z < -ballon_release_hgt) {
           balloon_released = true;
       }
       balloon_position += balloon_velocity * (1.0e-6f * (float)frame_time_us);
    }

    if (!balloon_released) {
        // assume a 50m tether with a 5Hz pogo frequency and damping ratio of 0.1
        Vector3f tether_pos_bf = Vector3f(-1.0f,0.0f,0.0f); // tether attaches to vehicle tail approx 1m behind c.g.
        const float tether_length = 50.0f;
        const float omega = 5.0*M_2PI; // rad/sec
        const  float zeta = 0.1f;
        float tether_stiffness = model.mass * sq(omega); // N/m
        float tether_damping = 2.0f * zeta *omega / model.mass; // N/(m/s)
        // NED relative position vector from tether attachment on plane to balloon attachment
        Vector3f relative_position = balloon_position - (position + dcm * tether_pos_bf);
        const float separation_distance = relative_position.length();
        if (separation_distance > tether_length) {
            // NED unit vector pointing from tether attachment on plane to attachment on balloon
            Vector3f tether_unit_vec_ef = relative_position.normalized();

            // NED velocity of attahment point on plane
            Vector3f attachment_velocity_ef = velocity_ef + dcm * (gyro % tether_pos_bf);

            // NED velocity of atachment point on balloon as seen by observer on attachemnt point on plane
            Vector3f relative_velocity = balloon_velocity - attachment_velocity_ef;

            // rate increase in separation between attachment point on plane and balloon
            float separation_speed = relative_velocity * tether_unit_vec_ef;

            // tension force in tether due to stiffness and damping
            float tension_force = MAX(0.0f, (separation_distance - tether_length) * tether_stiffness + separation_speed * tether_damping);

            Vector3f tension_force_vector_ef = tether_unit_vec_ef * tension_force;
            Vector3f tension_force_vector_bf = dcm.transposed() * tension_force_vector_ef;
            force += tension_force_vector_bf;

            Vector3f tension_moment_vector_bf = tether_pos_bf % tension_force_vector_bf;
            Vector3f tension_rot_accel = Vector3f(tension_moment_vector_bf.x/model.IXX, tension_moment_vector_bf.y/model.IYY, tension_moment_vector_bf.z/model.IZZ);
            rot_accel += tension_rot_accel;
        }
    }

    // scale thrust to match nose up hover throttle
    float thrust_scale = (model.mass * GRAVITY_MSS) / model.hoverThrottle;
    float thrust   = throttle * thrust_scale;

    accel_body = Vector3f(thrust, 0, 0) + force;
    accel_body /= model.mass;

    if (on_ground()) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
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
    
    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
