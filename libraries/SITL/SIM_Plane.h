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
#include "SIM_ICEngine.h"
#include <Filter/LowPassFilter.h>

namespace SITL {

/*
  a very simple plane simulator
 */
class Plane : public Aircraft {
public:
    Plane(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Plane(frame_str);
    }

protected:
    const float hover_throttle = 0.7f;
    float air_density = SSL_AIR_DENSITY; // kg/m^3 at sea level, ISA conditions
    float angle_of_attack;
    float beta;

    struct {
        // from last_letter skywalker_2013/aerodynamics.yaml
        // thanks to Georacer!
        float s = 0.45;
        float b = 1.88;
        float c = 0.24;
        float c_lift_0 = 0.56;
        float c_lift_deltae = 0;
        float c_lift_a = 6.9;
        float c_lift_q = 0;
        float mcoeff = 50;
        float oswald = 0.9;
        float alpha_stall = 0.4712;
        float c_drag_q = 0;
        float c_drag_deltae = 0.0;
        float c_drag_p = 0.0025;
        float c_y_0 = 0;
        float c_y_b = -0.98;
        float c_y_p = 0;
        float c_y_r = 0;
        float c_y_deltaa = 0;
        float c_y_deltar = -0.2;
        float c_l_0 = 0;
        float c_l_p = -1.0;
        float c_l_b = -0.12;
        float c_l_r = 0.14;
        float c_l_deltaa = 0.25;
        float c_l_deltar = -0.037;
        float c_m_0 = 0.045;
        float c_m_a = -0.7;
        float c_m_q = -20;
        float c_m_deltae = 1.0;
        float c_n_0 = 0;
        float c_n_b = 0.25;
        float c_n_p = 0.022;
        float c_n_r = -1;
        float c_n_deltaa = 0.00;
        float c_n_deltar = 0.1;
        float deltaa_max = 0.3491;
        float deltae_max = 0.3491;
        float deltar_max = 0.3491;
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        Vector3f CGOffset{-0.15, 0, -0.05};
    } coefficient;

    float thrust_scale;
    bool reverse_thrust;
    bool elevons;
    bool vtail;
    bool dspoilers;
    bool reverse_elevator_rudder;
    bool ice_engine;
    bool tailsitter;
    bool have_launcher;
    float launch_accel;
    float launch_time;

    const uint8_t throttle_servo = 2;
    const int8_t choke_servo = 14;
    const int8_t ignition_servo = 12;
    const int8_t starter_servo = 13;
    const float slewrate = 100;
    ICEngine icengine{
        throttle_servo,
        choke_servo,
        ignition_servo,
        starter_servo,
        slewrate
    };

    float alpharad;
    float betarad;

    /*
      parameters that define the multicopter model. Can be loaded from
      a json file to give a custom model
     */
    struct Model {
        // total vehicle mass
        float mass = 9.07441; // kg

        // reference area
        float Sref = 0.92762; // m^2

        float alphaRef = 0.0f; // angle of attack used as zero datum for aero derivatives (rad)

        float refSpan = 1.827411; // m
        float refChord = 0.507614; // m
        float IXX = 0.234; // kg-m^2
        float IYY = 1.85; // kg-m^2
        float IZZ = 2.04; // kg-m^2

        float Xcg = 0.0000000; // X coordinate of c.g. wrt the aero moment reference centre - positive is forward - m
        float Ycg = 0.0000000; // Y coordinate of c.g. wrt the aero moment reference centre - positive is right - m
        float Zcg = 0.0000000; // Z coordinate of c.g. wrt the aero moment reference centre - positive is down - m

        // tables containing AoA, CN and CA coefficient data
        uint8_t n_alpha = 0; // number of points - set to zero if no sweep data available
        float alpha_sweep_rad[25];
        float CA_sweep[25];
        float CN_sweep[25];

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
    } axis;

    struct ModelCFD {
      float mass = 220.0; // Kg
      float IXX = 20.0; // roll inertia about c.g. - Kg.m^2
      float IYY = 70.0; // pitch inertia about c.g. - Kg.m^2
      float IZZ = 70.0; // yaw inertia about c.g. - Kg.m^2
      float Sref = 1.4340000; // reference area for force and moment coefficients - m^2
      float Cref = 0.3620000; // scaling length for longitudinal moment coefficients - m
      float Bref = 1.9810000; // scaling length for lateral moment coefficients - m
      float Xref = 0.6044410; // X coordinate of moment reference centre - positive is back - m
      float Yref = 0.0000000; // Y coordinate of moment reference centre - positive is right - m
      float Zref = -0.0040840; // Z coordinate of moment reference centre - positive is up - m
      float Xcg = 0.6044410; // X coordinate of c.g. - positive is back - m
      float Ycg = 0.0000000; // Y coordinate of c.g. - positive is right - m
      float Zcg = 0.0000000; // Z coordinate of c.g. - positive is up - m
      float AoA_ref = 10.000000; // angle of attack used to generate Base_Aero data - deg
      float Beta_ref = 0.0000000; // angle fo sideslip used to generate Base_Aero data - deg
      float Vinf = 50.0000000; // true airspeed used to generate aero data - m/s

      float delta_alpha = radians(1.0); // angle of attack delta used to generate Alpha_Delta data - rad
      float delta_beta = radians(1.0); // angle of sideslip delta used to generate Beta_Delta data - rad
      float delta_roll_rate = 1.0f; // roll rate used to generate Roll_Rate_Delta data - rad/sec
      float delta_pitch_rate = 1.0f; // pitch rate used to generate Pitch_Rate_Delta data - rad/sec
      float delta_yaw_rate = 1.0f; // yaw rate used to generate Yaw_Rate_Delta data - rad/sec
      float delta_control = radians(1.0); // deflection used to generate control surface effectiveness data - rad

      // CFx and CFz vs alpha data derived from sweep data provided 2022-04-08
      // X is rearwards,Z is up in body frame
      uint8_t n_alpha = 25;
      float alpha_sweep_deg[25] = {-2.0,-1.0,0.0,0.5,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0};
      float CFx_sweep[25] = {0.156,0.166,0.164,0.168,0.172,0.172,0.168,0.162,0.158,0.152,0.144,0.139,0.129,0.116,0.107,0.092,0.078,0.064,0.056,0.030,0.013,0.003,-0.004,0.071,0.068};
      float CFz_sweep[25] = {0.138,0.197,0.236,0.284,0.339,0.409,0.466,0.512,0.576,0.640,0.708,0.782,0.844,0.906,0.977,1.039,1.092,1.164,1.211,1.274,1.319,1.356,1.369,1.240,1.262};

      // table data from PGB_19_DegenGeom.stab
      // X is rearwards, Y is right, Z is up in body frame
      // Moments are about moment reference centre
      //                                CFx          CFy          CFz          CMx          CMy          CMz
      float Base_Aero[6]         = {-0.0724402,  -0.0001758,   0.8455596,   0.0000542,  -0.1810522,  -0.0000879};
      float Alpha_Delta[6]       = {-0.0879221,  -0.0002347,   0.9021488,   0.0000479,  -0.2042463,  -0.0001332};
      float Beta_Delta[6]        = {-0.0724336,  -0.0284701,   0.8456362,   0.0062341,  -0.1821936,  -0.0142521};
      float Roll_Rate_Delta[6]   = {-0.0728790,  -0.0232658,   0.8458223,   0.0224272,  -0.1826518,  -0.0106415};
      float Pitch_Rate_Delta[6]  = {-0.0772657,  -0.0000892,   0.8791727,   0.0000201,  -0.3794061,  -0.0000496};
      float Yaw_Rate_Delta[6]    = {-0.0683952,   0.0341960,   0.8454556,  -0.0110672,  -0.1721448,   0.0207767};
      float Front_Right_Delta[6] = {-0.0722422,  -0.0009996,   0.8518152,   0.0021481,  -0.1701203,  -0.0004897};
      float Front_Left_Delta[6]  = {-0.0726025,  -0.0012347,   0.8392525,   0.0022508,  -0.1920116,  -0.0006566};
      float Rear_Right_Delta[6]  = {-0.0725716,   0.0002820,   0.8525984,   0.0021835,  -0.2051517,   0.0002638};
      float Rear_Left_Delta[6]   = {-0.0722704,   0.0002839,   0.8384862,   0.0021943,  -0.1568705,   0.0002782};

      // angle limits
      float aileronDeflectionLimitDeg = 15.0;
      float elevatorDeflectionLimitDeg = 15.0;
      float alphaMaxDeg = 12.0;
      float betaMaxDeg = 12.0;

      // mixing from aileron and elevator to individual control surfaces (front right, front left, rear right, rear left)
      // assume 50% aileron differential is used
      float ail_to_fr = - 0.5; // positive deflection in CFD model is TE down
      float ail_to_fl = - 1.0; // positive deflection is CFD model TE up
      float ail_to_rr = - 0.5; // positive deflection is CFD model TE down
      float ail_to_rl = - 1.0; // positive deflection is CFD model TE up
      float ele_to_fr = + 0.5; // positive deflection is CFD model TE down
      float ele_to_fl = - 1.0; // positive deflection is CFD model TE up
      float ele_to_rr = - 0.5; // positive deflection is CFD model TE down
      float ele_to_rl = + 1.0; // positive deflection is CFD model TE up

    } default_cfd_model;

    struct ModelCFD cfd_model;

    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    void convert_cfd_data(ModelCFD &cfd);
};

} // namespace SITL
