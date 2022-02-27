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
  simple plane simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

#define USE_CFD_DATA true

namespace SITL {

/*
  a very simple plane simulator
 */
class PlaneJSON : public Aircraft {
public:
    PlaneJSON(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new PlaneJSON(frame_str);
    }

    bool on_ground() const override;

protected:
    const float hover_throttle = 0.7f;
    float alpharad;
    float betarad;

    /*
      parameters that define the multicopter model. Can be loaded from
      a json file to give a custom model
     */
    const struct Model {
        // total vehicle mass
        float mass = 9.07441; // kg

        // reference area
        float Sref = 0.92762; // m^2

        float refSpan = 1.827411; // m
        float refChord = 0.507614; // m
        float IXX = 0.234; // kg-m^2
        float IYY = 1.85; // kg-m^2
        float IZZ = 2.04; // kg-m^2

        // CN is coefficients for forces on +Z axis
        // quadratic in alpharad
        float CN2 = -0.5771;
        float CN1 = 3.9496;
        float CN0 = 0;

        // CA is the coefficients for forces on +X axis
        // quadratic in alpharad
        float CA2 = -1.6809;
        float CA1 = -0.0057;
        float CA0 = 0.0150;

        // CY is the coefficients for forces on the +Y axis
        // quadratic in alpharad, with betarad factor
        float CY2 = -3.342;
        float CY1 = 0.0227;
        float CY0 = -0.4608;

        // Cl is the coefficients for moments on X axis
        // quadratic in alpharad, with betarad factor
        float Cl2 = 0.2888;
        float Cl1 = -0.8518;
        float Cl0 = -0.0491;

        // Cm is the coefficients for moments on Y axis
        // quadratic in alpharad
        float Cm2 = 0.099;
        float Cm1 = -0.6506;
        float Cm0 = -0.0005;

        // Cn is the coefficients for moments on Z axis
        // quadratic in alpharad, with betarad factor
        float Cn2 = 0.0057;
        float Cn1 = -0.0101;
        float Cn0 = 0.1744;

        // controls neutral dynamic derivatives
        // p, q, r are gyro rates
        float Cmq = -6.1866;

        float Clp2 = 0.156;
        float Clp1 = 0.0129;
        float Clp0 = -0.315;

        float Clr2 = -0.0284;
        float Clr1 = 0.2641;
        float Clr0 = 0.0343;

        float Cnp2 = 0.0199;
        float Cnp1 = -0.315;
        float Cnp0 = -0.013;

        float Cnr2 = 0.1297;
        float Cnr1 = 0.0343;
        float Cnr0 = -0.264;

        // elevator
        float elevatorDeflectionLimitDeg = -12.5;
        float deltaCNperRadianElev = -0.7;
        float deltaCAperRadianElev = 0.12;
        float deltaCmperRadianElev = 1.39;
        float deltaCYperRadianElev = 0;
        float deltaClperRadianElev = 0;
        float deltaCnperRadianElev = 0;

        // rudder
        float rudderDeflectionLimitDeg = 18.0;
        float deltaCNperRadianRud = 0;
        float deltaCAperRadianRud = 0.058;
        float deltaCmperRadianRud = 0;
        float deltaCYperRadianRud = 0.31;
        float deltaClperRadianRud = 0.038;
        float deltaCnperRadianRud = -0.174;

        // aileron
        float aileronDeflectionLimitDeg = 15.5;
        float deltaCNperRadianAil = 0;
        float deltaCAperRadianAil = 0.016;
        float deltaCmperRadianAil = 0;
        float deltaCYperRadianAil = -0.015;

        // quadratic in alpharad
        float deltaClperRadianAil0 = 0.09191;
        float deltaClperRadianAil1 = 0.0001;
        float deltaClperRadianAil2 = -0.08645;

        // quadratic in alpharad
        float deltaCnperRadianAil0 = 0.00789;
        float deltaCnperRadianAil1 = 0.00773;
        float deltaCnperRadianAil2 = -0.01162;

        // Forces in the +X direction are –CA * q * Sref
        // Forces in the +Y direction are  +CY * q * Sref
        // Forces in the +Z direction are  –CN * q *Sref
        // Moments about the X axis are +Cl * q * Sref * RefSpan
        // Moments about the Y axis are +Cm * q * Sref * RefChord
        // Moments about the Z axis are +Cn * q * Sref * RefSpan

        float hoverThrottle = 2.0;

        // low altitude
        float alphaRadMax = 0.209;
        float betaRadMax = 0.209;

        // balloon launch parameters
        float tetherLength = 50.0f;       // length of tether from balloon to aircraft (m)
        float tetherPogoFreq = 2.0f;      // measured vertical frequency of on tether (Hz)

    } default_model;

    struct Model model;

      enum {
          CFx = 0,
          CFy,
          CFz,
          CMx,
          CMy,
          CMz,
      } index;

    struct ModelCFD {
      float mass = 220.0; // Kg
      float IXX = 20.0; // roll inertial - Kg.m^2
      float IYY = 70.0; // pitch inertia - Kg.m^2
      float IZZ = 70.0; // yaw inertia - Kg.m^2
      float Sref = 1.4340000; // reference area for force and moment coefficients - m^2
      float Cref = 0.3620000; // scaling length for longitudinal moment coefficients - m
      float Bref = 1.9810000; // scaling length for lateral moment coefficients - m
      float Xcg = 0.6044410; // m positive is rearwards
      float Ycg = 0.0000000; // m positive is right
      float Zcg = -0.0040840; // m positive is up
      float AoA_ref = 2.7000000; // angle of attack used to generate Base_Aero data - deg
      float Beta_ref = 0.0000000; // angle fo sideslip used to generate Base_Aero data - deg
      float Vinf = 50.0000000; // true airspeed used to generate aero data - m/s

      float delta_alpha = radians(1.0); // angle of attack delta used to generate Alpha_Delta data - rad
      float delta_beta = radians(1.0); // angle of sideslip delta used to generate Beta_Delta data - rad
      float delta_roll_rate = 1.0f; // roll rate used to generate Roll_Rate_Delta data - rad/sec
      float delta_pitch_rate = 1.0f; // pitch rate used to generate Pitch_Rate_Delta data - rad/sec
      float delta_yaw_rate = 1.0f; // yaw rate used to generate Yaw_Rate_Delta data - rad/sec
      float delta_aileron = radians(1.0); // aileron deflection used to generate Aileron_Delta data - rad
      float delta_elevator = radians(1.0); // elevator deflection used to generate Elevator_Delta data - rad
      float delta_rudder = radians(1.0); // rudder deflection used to generate Rudder_Delta data - rad

      // X is rearwards, Y is right, Z is up in body frame
      //                               CFx          CFy          CFz          CMx          CMy          CMz
      float Base_Aero[6]        = { 0.0002381,   0.0000358,   0.4192856,  -0.0000138,  -0.0001385,   0.0000198}; 
      float Alpha_Delta[6]      = {-0.0052591,   0.0000090,   0.4782762,  -0.0000073,  -0.0232671,   0.0000054};
      float Beta_Delta[6]       = {-0.0000403,  -0.0259597,   0.4192175,   0.0057064,  -0.0007883,  -0.0129232};
      float Roll_Rate_Delta[6]  = {-0.0002035,  -0.0084585,   0.4193562,   0.0177141,  -0.0003815,  -0.0030445};
      float Pitch_Rate_Delta[6] = {-0.0009847,  -0.0000064,   0.4530610,  -0.0000007,  -0.2038634,  -0.0000027};
      float Yaw_Rate_Delta[6]   = {-0.0003137,   0.0415058,   0.4193819,  -0.0114662,  -0.0011315,   0.0274757};
      float Aileron_Delta[6]    = { 0.0002381,   0.0000358,   0.4192856,  -0.0000138,  -0.0001385,   0.0000198}; 
      float Elevator_Delta[6]   = { 0.0002381,   0.0000358,   0.4192856,  -0.0000138,  -0.0001385,   0.0000198}; 
      float Rudder_Delta[6]     = { 0.0002381,   0.0000358,   0.4192856,  -0.0000138,  -0.0001385,   0.0000198}; 

      // angle limits
      float aileronDeflectionLimitDeg = 15.0;
      float elevatorDeflectionLimitDeg = 15.0;
      float rudderDeflectionLimitDeg = 15.0;
      float alphaMaxDeg = 12.0;
      float betaMaxDeg = 12.0;

    } default_cfd_model;

    struct ModelCFD cfd_model;

    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, const Vector3f &force) const;
    bool update_balloon(float balloon, Vector3f &force, Vector3f &rot_accel);
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    Model convert_cfd_data(ModelCFD &cfd_model);

    Vector3f balloon_velocity;           // balloon velocity NED
    Vector3f balloon_position{0.0f, 0.0f, -45.0f}; // balloon position NED from origin

    enum class carriageState {
        NONE = 0, // no carriage option available
        WAITING_FOR_PICKUP = 1, // in launch cradle waiting to be picked up by launch vehicle
        WAITING_FOR_RELEASE = 2, // being carried by luanch vehicle waitng to be released
        PRE_RELEASE = 3, // had been released by launch vehicle
        RELEASED = 4 // had been released by launch vehicle
    } carriage_state;
    bool plane_air_release;    // true when plane has separated from the airborne launching platform
};

} // namespace SITL
