#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

/*
 * Fuse true airspeed measurements using explicit algebraic equations auto-generated from
 * libraries/AP_NavEKF3/python/main.py with output recorded in libraries/AP_NavEKF3/python/tas_generated.cpp
*/
void NavEKF3_core::FuseAirspeed()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseAirspeed);

    // declarations
    float &vn = stateStruct.velocity.x;
    float &ve = stateStruct.velocity.y;
    float &vd = stateStruct.velocity.z;
    float &vwn = stateStruct.wind_vel.x;
    float &vwe = stateStruct.wind_vel.y;
    float EAS2TAS = _ahrs->get_EAS2TAS();
    const float R_TAS = sq(constrain_float(frontend->_easNoise, 0.5f, 5.0f) * constrain_float(EAS2TAS, 0.9f, 10.0f));
    Vector24 H_TAS;
    float VtasPred;
    const bool airDataFusionWindOnly = false; // place holder for future functionality

    // health is set bad until test passed
    tasHealth = false;

    // calculate the predicted airspeed
    VtasPred = norm((ve - vwe) , (vn - vwn) , vd);
    // perform fusion of True Airspeed measurement
    if (VtasPred > 1.0f)
    {
        // TAS observatoin Jacobian, innovation variance and kalman gain
        float HK0 = vn - vwn;
        float HK1 = ve - vwe;
        float HK2 = 1.0f / VtasPred;
        float HK3 = HK0*HK2;
        float HK4 = HK1*HK2;
        float HK5 = HK2*vd;
        float HK6 = HK2*(-vn + vwn);
        float HK7 = HK2*(-ve + vwe);
        float HK8 = HK3*P[4][6] + HK4*P[5][6] + HK5*P[6][6] + HK6*P[6][22] + HK7*P[6][23];
        float HK9 = HK3*P[4][23] + HK4*P[5][23] + HK5*P[6][23] + HK6*P[22][23] + HK7*P[23][23];
        float HK10 = HK3*P[4][5] + HK4*P[5][5] + HK5*P[5][6] + HK6*P[5][22] + HK7*P[5][23];
        float HK11 = HK3*P[4][22] + HK4*P[5][22] + HK5*P[6][22] + HK6*P[22][22] + HK7*P[22][23];
        float HK12 = HK3*P[4][4] + HK4*P[4][5] + HK5*P[4][6] + HK6*P[4][22] + HK7*P[4][23];
        float HK13;

        H_TAS[0] = 0;
        H_TAS[1] = 0;
        H_TAS[2] = 0;
        H_TAS[3] = 0;
        H_TAS[4] = HK3;
        H_TAS[5] = HK4;
        H_TAS[6] = HK5;
        H_TAS[7] = 0;
        H_TAS[8] = 0;
        H_TAS[9] = 0;
        H_TAS[10] = 0;
        H_TAS[11] = 0;
        H_TAS[12] = 0;
        H_TAS[13] = 0;
        H_TAS[14] = 0;
        H_TAS[15] = 0;
        H_TAS[16] = 0;
        H_TAS[17] = 0;
        H_TAS[18] = 0;
        H_TAS[19] = 0;
        H_TAS[20] = 0;
        H_TAS[21] = 0;
        H_TAS[22] = HK6;
        H_TAS[23] = HK7;

        // calculate Kalman gains
        varInnovVtas = (HK10*HK4 + HK11*HK6 + HK12*HK3 + HK5*HK8 + HK7*HK9 + R_TAS);
        if (varInnovVtas >= R_TAS) {
            HK13 = 1.0f / varInnovVtas;
            faultStatus.bad_airspeed = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_airspeed = true;
            return;
        }

        if (!airDataFusionWindOnly) {
            Kfusion[0] = HK13*(HK3*P[0][4] + HK4*P[0][5] + HK5*P[0][6] + HK6*P[0][22] + HK7*P[0][23]);
            Kfusion[1] = HK13*(HK3*P[1][4] + HK4*P[1][5] + HK5*P[1][6] + HK6*P[1][22] + HK7*P[1][23]);
            Kfusion[2] = HK13*(HK3*P[2][4] + HK4*P[2][5] + HK5*P[2][6] + HK6*P[2][22] + HK7*P[2][23]);
            Kfusion[3] = HK13*(HK3*P[3][4] + HK4*P[3][5] + HK5*P[3][6] + HK6*P[3][22] + HK7*P[3][23]);
            Kfusion[4] = HK12*HK13;
            Kfusion[5] = HK10*HK13;
            Kfusion[6] = HK13*HK8;
            Kfusion[7] = HK13*(HK3*P[4][7] + HK4*P[5][7] + HK5*P[6][7] + HK6*P[7][22] + HK7*P[7][23]);
            Kfusion[8] = HK13*(HK3*P[4][8] + HK4*P[5][8] + HK5*P[6][8] + HK6*P[8][22] + HK7*P[8][23]);
            Kfusion[9] = HK13*(HK3*P[4][9] + HK4*P[5][9] + HK5*P[6][9] + HK6*P[9][22] + HK7*P[9][23]);
        } else {
            // zero indexes 0 to 9 = 10*4 bytes
            memset(&Kfusion[0], 0, 40);
        }

        if (!inhibitDelAngBiasStates && !airDataFusionWindOnly) {
            Kfusion[10] = HK13*(HK3*P[4][10] + HK4*P[5][10] + HK5*P[6][10] + HK6*P[10][22] + HK7*P[10][23]);
            Kfusion[11] = HK13*(HK3*P[4][11] + HK4*P[5][11] + HK5*P[6][11] + HK6*P[11][22] + HK7*P[11][23]);
            Kfusion[12] = HK13*(HK3*P[4][12] + HK4*P[5][12] + HK5*P[6][12] + HK6*P[12][22] + HK7*P[12][23]);
        } else {
            // zero indexes 10 to 12 = 3*4 bytes
            memset(&Kfusion[10], 0, 12);
        }

        if (!inhibitDelVelBiasStates && !airDataFusionWindOnly) {
            Kfusion[13] = HK13*(HK3*P[4][13] + HK4*P[5][13] + HK5*P[6][13] + HK6*P[13][22] + HK7*P[13][23]);
            Kfusion[14] = HK13*(HK3*P[4][14] + HK4*P[5][14] + HK5*P[6][14] + HK6*P[14][22] + HK7*P[14][23]);
            Kfusion[15] = HK13*(HK3*P[4][15] + HK4*P[5][15] + HK5*P[6][15] + HK6*P[15][22] + HK7*P[15][23]);
        } else {
            // zero indexes 13 to 15 = 3*4 bytes
            memset(&Kfusion[13], 0, 12);
        }

        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates && !airDataFusionWindOnly) {
            Kfusion[16] = HK13*(HK3*P[4][16] + HK4*P[5][16] + HK5*P[6][16] + HK6*P[16][22] + HK7*P[16][23]);
            Kfusion[17] = HK13*(HK3*P[4][17] + HK4*P[5][17] + HK5*P[6][17] + HK6*P[17][22] + HK7*P[17][23]);
            Kfusion[18] = HK13*(HK3*P[4][18] + HK4*P[5][18] + HK5*P[6][18] + HK6*P[18][22] + HK7*P[18][23]);
            Kfusion[19] = HK13*(HK3*P[4][19] + HK4*P[5][19] + HK5*P[6][19] + HK6*P[19][22] + HK7*P[19][23]);
            Kfusion[20] = HK13*(HK3*P[4][20] + HK4*P[5][20] + HK5*P[6][20] + HK6*P[20][22] + HK7*P[20][23]);
            Kfusion[21] = HK13*(HK3*P[4][21] + HK4*P[5][21] + HK5*P[6][21] + HK6*P[21][22] + HK7*P[21][23]);
        } else {
            // zero indexes 16 to 21 = 6*4 bytes
            memset(&Kfusion[16], 0, 24);
        }

        if (!inhibitWindStates) {
            Kfusion[22] = HK11*HK13;
            Kfusion[23] = HK13*HK9;
        } else {
            // zero indexes 22 to 23 = 2*4 bytes
            memset(&Kfusion[22], 0, 8);
        }

        // calculate measurement innovation
        innovVtas = VtasPred - tasDataDelayed.tas;

        // calculate the innovation consistency test ratio
        tasTestRatio = sq(innovVtas) / (sq(MAX(0.01f * (float)frontend->_tasInnovGate, 1.0f)) * varInnovVtas);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        tasHealth = ((tasTestRatio < 1.0f) || badIMUdata);
        tasTimeout = (imuSampleTime_ms - lastTasPassTime_ms) > frontend->tasRetryTime_ms;

        // test the ratio before fusing data, forcing fusion if airspeed and position are timed out as we have no choice but to try and use airspeed to constrain error growth
        if (tasHealth || (tasTimeout && posTimeout)) {

            // restart the counter
            lastTasPassTime_ms = imuSampleTime_ms;

            // correct the state vector
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                statesArray[j] = statesArray[j] - Kfusion[j] * innovVtas;
            }
            stateStruct.quat.normalize();

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=3; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 4; j<=6; j++) {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (unsigned j = 7; j<=21; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 22; j<=23; j++) {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
            }
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                for (unsigned i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][4] * P[4][j];
                    res += KH[i][5] * P[5][j];
                    res += KH[i][6] * P[6][j];
                    res += KH[i][22] * P[22][j];
                    res += KH[i][23] * P[23][j];
                    KHP[i][j] = res;
                }
            }
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=stateIndexLim; j++) {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop performance timer
    hal.util->perf_end(_perf_FuseAirspeed);
}

// select fusion of true airspeed measurements
void NavEKF3_core::SelectTasFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data locking out fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !airSpdFusionDelayed) {
        airSpdFusionDelayed = true;
        return;
    } else {
        airSpdFusionDelayed = false;
    }

    // get true airspeed measurement
    readAirSpdData();

    // If we haven't received airspeed data for a while, then declare the airspeed data as being timed out
    if (imuSampleTime_ms - tasDataNew.time_ms > frontend->tasRetryTime_ms) {
        tasTimeout = true;
    }

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then perform TAS fusion
    if (tasDataToFuse && statesInitialised && !inhibitWindStates) {
        FuseAirspeed();
        prevTasStep_ms = imuSampleTime_ms;
    }
}


// select fusion of synthetic sideslip measurements
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// it requires a stable wind for best results and should not be used for aerobatic flight with manoeuvres that induce large sidslip angles (eg knife-edge, spins, etc)
void NavEKF3_core::SelectBetaFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !sideSlipFusionDelayed) {
        sideSlipFusionDelayed = true;
        return;
    } else {
        sideSlipFusionDelayed = false;
    }

    // set true when the fusion time interval has triggered
    bool f_timeTrigger = ((imuSampleTime_ms - prevBetaStep_ms) >= frontend->betaAvg_ms);
    // set true when use of synthetic sideslip fusion is necessary because we have limited sensor data or are dead reckoning position
    bool f_required = !(use_compass() && useAirspeed() && ((imuSampleTime_ms - lastPosPassTime_ms) < frontend->posRetryTimeNoVel_ms));
    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    bool f_feasible = (assume_zero_sideslip() && !inhibitWindStates);
    // use synthetic sideslip fusion if feasible, required and enough time has lapsed since the last fusion
    if (f_feasible && f_required && f_timeTrigger) {
        FuseSideslip();
        prevBetaStep_ms = imuSampleTime_ms;
    }
}

/*
 * Fuse sythetic sideslip measurement of zero using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseSideslip()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseSideslip);

    // declarations
    const float R_BETA = sq(radians(10.0f));
    Vector3f vel_rel_wind;
    Vector24 H_BETA;
    float innovBeta;
    const bool airDataFusionWindOnly = false; // place holder for future functionality

    // local variable names required for state access
    const float &q0 = stateStruct.quat[0];
    const float &q1 = stateStruct.quat[1];
    const float &q2 = stateStruct.quat[2];
    const float &q3 = stateStruct.quat[3];
    const float &vn = stateStruct.velocity.x;
    const float &ve = stateStruct.velocity.y;
    const float &vd = stateStruct.velocity.z;
    const float &vwn = stateStruct.wind_vel.x;
    const float &vwe = stateStruct.wind_vel.y;

    // calculate predicted wind relative velocity in NED
    vel_rel_wind.x = vn - vwn;
    vel_rel_wind.y = ve - vwe;
    vel_rel_wind.z = vd;

    // rotate into body axes
    vel_rel_wind = prevTnb * vel_rel_wind;

    // perform fusion of assumed sideslip  = 0
    if (vel_rel_wind.x > 5.0f)
    {
        // calculate sub-expressions
        const float S0 = 2*q0;
        const float S1 = 2*q3;
        const float S2 = S0*q1 + S1*q2;
        const float S3 = S0*q2;
        const float S4 = 2*q1;
        const float S5 = S4*q3;
        const float S6 = ve - vwe;
        const float S7 = S0*q3;
        const float S8 = S4*q2;
        const float S9 = S7 + S8;
        const float S10 = vn - vwn;
        const float S11 = sq(q0);
        const float S12 = sq(q3);
        const float S13 = S11 - S12;
        const float S14 = sq(q1);
        const float S15 = sq(q2);
        const float S16 = S14 - S15;
        const float S17 = S13 + S16;
        const float S18 = S10*S17 + S6*S9 + vd*(-S3 + S5);
        if (!is_positive(sq(S18))) {
            faultStatus.bad_sideslip = true;
            return;
        }
        faultStatus.bad_sideslip = false;
        const float S19 = 1.0F/S18;
        const float S20 = -S7;
        const float S21 = S20 + S8;
        const float S22 = -S14 + S15;
        const float S23 = S13 + S22;
        const float S24 = (S10*S21 + S2*vd + S23*S6)/sq(S18);
        const float S25 = S19*S2 + S24*(S3 - S5);
        const float S26 = -S11 + S12;
        const float S27 = S19*S21 + S24*(S22 + S26);
        const float S28 = -S8;
        const float S29 = S19*S23 + S24*(S20 + S28);
        const float S30 = S17*S24 + S19*(S28 + S7);
        const float S31 = S19*(S16 + S26) + S24*S9;
        const float S32 = S4*vd;
        const float S33 = S0*S6;
        const float S34 = S1*S10;
        const float S35 = 2*q2;
        const float S36 = -S0*S10 - S1*S6 + S35*vd;
        const float S37 = S19*(S32 + S33 - S34) + S24*S36;
        const float S38 = S0*vd + S10*S35 - S4*S6;
        const float S39 = S1*vd;
        const float S40 = S10*S4;
        const float S41 = S35*S6;
        const float S42 = S19*S38 + S24*(-S39 - S40 - S41);
        const float S43 = S19*(S39 + S40 + S41) + S24*S38;
        const float S44 = S19*S36 + S24*(-S32 - S33 + S34);

        // calculate innovation variance and check numnerical conditioning
        const float innov_var = R_BETA + S25*(P[0][6]*S37 + P[1][6]*S42 + P[2][6]*S43 + P[3][6]*S44 + P[4][6]*S27 + P[5][6]*S29 + P[6][22]*S30 + P[6][23]*S31 + P[6][6]*S25) + S27*(P[0][4]*S37 + P[1][4]*S42 + P[2][4]*S43 + P[3][4]*S44 + P[4][22]*S30 + P[4][23]*S31 + P[4][4]*S27 + P[4][5]*S29 + P[4][6]*S25) + S29*(P[0][5]*S37 + P[1][5]*S42 + P[2][5]*S43 + P[3][5]*S44 + P[4][5]*S27 + P[5][22]*S30 + P[5][23]*S31 + P[5][5]*S29 + P[5][6]*S25) + S30*(P[0][22]*S37 + P[1][22]*S42 + P[22][22]*S30 + P[22][23]*S31 + P[2][22]*S43 + P[3][22]*S44 + P[4][22]*S27 + P[5][22]*S29 + P[6][22]*S25) + S31*(P[0][23]*S37 + P[1][23]*S42 + P[22][23]*S30 + P[23][23]*S31 + P[2][23]*S43 + P[3][23]*S44 + P[4][23]*S27 + P[5][23]*S29 + P[6][23]*S25) + S37*(P[0][0]*S37 + P[0][1]*S42 + P[0][22]*S30 + P[0][23]*S31 + P[0][2]*S43 + P[0][3]*S44 + P[0][4]*S27 + P[0][5]*S29 + P[0][6]*S25) + S42*(P[0][1]*S37 + P[1][1]*S42 + P[1][22]*S30 + P[1][23]*S31 + P[1][2]*S43 + P[1][3]*S44 + P[1][4]*S27 + P[1][5]*S29 + P[1][6]*S25) + S43*(P[0][2]*S37 + P[1][2]*S42 + P[2][22]*S30 + P[2][23]*S31 + P[2][2]*S43 + P[2][3]*S44 + P[2][4]*S27 + P[2][5]*S29 + P[2][6]*S25) + S44*(P[0][3]*S37 + P[1][3]*S42 + P[2][3]*S43 + P[3][22]*S30 + P[3][23]*S31 + P[3][3]*S44 + P[3][4]*S27 + P[3][5]*S29 + P[3][6]*S25);
        if (innov_var < R_BETA) {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_sideslip = true;
            return;
        }

        // sideslip observation matrix and kalman gain
        const float HK0 = 2*q1;
        const float HK1 = HK0*vd;
        const float HK2 = ve - vwe;
        const float HK3 = 2*q0;
        const float HK4 = HK2*HK3;
        const float HK5 = vn - vwn;
        const float HK6 = 2*q3;
        const float HK7 = HK5*HK6;
        const float HK8 = HK3*q2;
        const float HK9 = HK0*q3;
        const float HK10 = HK3*q3;
        const float HK11 = HK0*q2;
        const float HK12 = HK10 + HK11;
        const float HK13 = sq(q0);
        const float HK14 = sq(q3);
        const float HK15 = HK13 - HK14;
        const float HK16 = sq(q1);
        const float HK17 = sq(q2);
        const float HK18 = HK16 - HK17;
        const float HK19 = HK15 + HK18;
        const float HK20 = HK12*HK2 + HK19*HK5 + vd*(-HK8 + HK9);
        const float HK21 = 1.0F/HK20;
        const float HK22 = 2*q2;
        const float HK23 = -HK2*HK6 + HK22*vd - HK3*HK5;
        const float HK24 = HK3*q1 + HK6*q2;
        const float HK25 = -HK10;
        const float HK26 = HK11 + HK25;
        const float HK27 = -HK16 + HK17;
        const float HK28 = HK15 + HK27;
        const float HK29 = (HK2*HK28 + HK24*vd + HK26*HK5)/sq(HK20);
        const float HK30 = HK21*(HK1 + HK4 - HK7) + HK23*HK29;
        const float HK31 = -HK0*HK2 + HK22*HK5 + HK3*vd;
        const float HK32 = HK6*vd;
        const float HK33 = HK0*HK5;
        const float HK34 = HK2*HK22;
        const float HK35 = HK21*HK31 + HK29*(-HK32 - HK33 - HK34);
        const float HK36 = HK21*(HK32 + HK33 + HK34) + HK29*HK31;
        const float HK37 = HK21*HK23 + HK29*(-HK1 - HK4 + HK7);
        const float HK38 = -HK13 + HK14;
        const float HK39 = HK21*HK26 + HK29*(HK27 + HK38);
        const float HK40 = -HK11;
        const float HK41 = HK21*HK28 + HK29*(HK25 + HK40);
        const float HK42 = HK21*HK24 + HK29*(HK8 - HK9);
        const float HK43 = HK19*HK29 + HK21*(HK10 + HK40);
        const float HK44 = HK12*HK29 + HK21*(HK18 + HK38);
        const float HK45 = HK30*P[0][0] + HK35*P[0][1] + HK36*P[0][2] + HK37*P[0][3] + HK39*P[0][4] + HK41*P[0][5] + HK42*P[0][6] + HK43*P[0][22] + HK44*P[0][23];
        const float HK46 = HK30*P[0][6] + HK35*P[1][6] + HK36*P[2][6] + HK37*P[3][6] + HK39*P[4][6] + HK41*P[5][6] + HK42*P[6][6] + HK43*P[6][22] + HK44*P[6][23];
        const float HK47 = HK30*P[0][4] + HK35*P[1][4] + HK36*P[2][4] + HK37*P[3][4] + HK39*P[4][4] + HK41*P[4][5] + HK42*P[4][6] + HK43*P[4][22] + HK44*P[4][23];
        const float HK48 = HK30*P[0][22] + HK35*P[1][22] + HK36*P[2][22] + HK37*P[3][22] + HK39*P[4][22] + HK41*P[5][22] + HK42*P[6][22] + HK43*P[22][22] + HK44*P[22][23];
        const float HK49 = HK30*P[0][5] + HK35*P[1][5] + HK36*P[2][5] + HK37*P[3][5] + HK39*P[4][5] + HK41*P[5][5] + HK42*P[5][6] + HK43*P[5][22] + HK44*P[5][23];
        const float HK50 = HK30*P[0][23] + HK35*P[1][23] + HK36*P[2][23] + HK37*P[3][23] + HK39*P[4][23] + HK41*P[5][23] + HK42*P[6][23] + HK43*P[22][23] + HK44*P[23][23];
        const float HK51 = HK30*P[0][1] + HK35*P[1][1] + HK36*P[1][2] + HK37*P[1][3] + HK39*P[1][4] + HK41*P[1][5] + HK42*P[1][6] + HK43*P[1][22] + HK44*P[1][23];
        const float HK52 = HK30*P[0][2] + HK35*P[1][2] + HK36*P[2][2] + HK37*P[2][3] + HK39*P[2][4] + HK41*P[2][5] + HK42*P[2][6] + HK43*P[2][22] + HK44*P[2][23];
        const float HK53 = HK30*P[0][3] + HK35*P[1][3] + HK36*P[2][3] + HK37*P[3][3] + HK39*P[3][4] + HK41*P[3][5] + HK42*P[3][6] + HK43*P[3][22] + HK44*P[3][23];
        const float HK54 = 1.0F/(HK30*HK45 + HK35*HK51 + HK36*HK52 + HK37*HK53 + HK39*HK47 + HK41*HK49 + HK42*HK46 + HK43*HK48 + HK44*HK50 + R_BETA);

        H_BETA[0] = HK30;
        H_BETA[1] = HK35;
        H_BETA[2] = HK36;
        H_BETA[3] = HK37;
        H_BETA[4] = HK39;
        H_BETA[5] = HK41;
        H_BETA[6] = HK42;
        memset(&H_BETA[7], 0, 60); // zero indexes 7 to 21 = 15*4 bytes
        H_BETA[22] = HK43;
        H_BETA[23] = HK44;

        // Calculate Kalman gains
        // Sideslip should be used to constrain yaw if navigating without a independent source of yaw
        const bool updateAllStates = !airDataFusionWindOnly ||
                                     ((effectiveMagCal != MagCal::EXTERNAL_YAW) &&
                                     (effectiveMagCal != MagCal::EXTERNAL_YAW_FALLBACK) &&
                                     !use_compass());

        if (updateAllStates) {
            Kfusion[0] = HK45*HK54;
            Kfusion[1] = HK51*HK54;
            Kfusion[2] = HK52*HK54;
            Kfusion[3] = HK53*HK54;
            Kfusion[4] = HK47*HK54;
            Kfusion[5] = HK49*HK54;
            Kfusion[6] = HK46*HK54;
            Kfusion[7] = HK54*(HK30*P[0][7] + HK35*P[1][7] + HK36*P[2][7] + HK37*P[3][7] + HK39*P[4][7] + HK41*P[5][7] + HK42*P[6][7] + HK43*P[7][22] + HK44*P[7][23]);
            Kfusion[8] = HK54*(HK30*P[0][8] + HK35*P[1][8] + HK36*P[2][8] + HK37*P[3][8] + HK39*P[4][8] + HK41*P[5][8] + HK42*P[6][8] + HK43*P[8][22] + HK44*P[8][23]);
            Kfusion[9] = HK54*(HK30*P[0][9] + HK35*P[1][9] + HK36*P[2][9] + HK37*P[3][9] + HK39*P[4][9] + HK41*P[5][9] + HK42*P[6][9] + HK43*P[9][22] + HK44*P[9][23]);
        } else {
            // zero indexes 0 to 9 = 10*4 bytes
            memset(&Kfusion[0], 0, 40);
        }

        if (!inhibitDelAngBiasStates && updateAllStates) {
            Kfusion[10] = HK54*(HK30*P[0][10] + HK35*P[1][10] + HK36*P[2][10] + HK37*P[3][10] + HK39*P[4][10] + HK41*P[5][10] + HK42*P[6][10] + HK43*P[10][22] + HK44*P[10][23]);
            Kfusion[11] = HK54*(HK30*P[0][11] + HK35*P[1][11] + HK36*P[2][11] + HK37*P[3][11] + HK39*P[4][11] + HK41*P[5][11] + HK42*P[6][11] + HK43*P[11][22] + HK44*P[11][23]);
            Kfusion[12] = HK54*(HK30*P[0][12] + HK35*P[1][12] + HK36*P[2][12] + HK37*P[3][12] + HK39*P[4][12] + HK41*P[5][12] + HK42*P[6][12] + HK43*P[12][22] + HK44*P[12][23]);
        } else {
            // zero indexes 10 to 12 = 3*4 bytes
            memset(&Kfusion[10], 0, 12);
        }

        if (!inhibitDelVelBiasStates && updateAllStates) {
            Kfusion[13] = HK54*(HK30*P[0][13] + HK35*P[1][13] + HK36*P[2][13] + HK37*P[3][13] + HK39*P[4][13] + HK41*P[5][13] + HK42*P[6][13] + HK43*P[13][22] + HK44*P[13][23]);
            Kfusion[14] = HK54*(HK30*P[0][14] + HK35*P[1][14] + HK36*P[2][14] + HK37*P[3][14] + HK39*P[4][14] + HK41*P[5][14] + HK42*P[6][14] + HK43*P[14][22] + HK44*P[14][23]);
            Kfusion[15] = HK54*(HK30*P[0][15] + HK35*P[1][15] + HK36*P[2][15] + HK37*P[3][15] + HK39*P[4][15] + HK41*P[5][15] + HK42*P[6][15] + HK43*P[15][22] + HK44*P[15][23]);
        } else {
            // zero indexes 13 to 15 = 3*4 bytes
            memset(&Kfusion[13], 0, 12);
        }

        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates && updateAllStates) {
            Kfusion[16] = HK54*(HK30*P[0][16] + HK35*P[1][16] + HK36*P[2][16] + HK37*P[3][16] + HK39*P[4][16] + HK41*P[5][16] + HK42*P[6][16] + HK43*P[16][22] + HK44*P[16][23]);
            Kfusion[17] = HK54*(HK30*P[0][17] + HK35*P[1][17] + HK36*P[2][17] + HK37*P[3][17] + HK39*P[4][17] + HK41*P[5][17] + HK42*P[6][17] + HK43*P[17][22] + HK44*P[17][23]);
            Kfusion[18] = HK54*(HK30*P[0][18] + HK35*P[1][18] + HK36*P[2][18] + HK37*P[3][18] + HK39*P[4][18] + HK41*P[5][18] + HK42*P[6][18] + HK43*P[18][22] + HK44*P[18][23]);
            Kfusion[19] = HK54*(HK30*P[0][19] + HK35*P[1][19] + HK36*P[2][19] + HK37*P[3][19] + HK39*P[4][19] + HK41*P[5][19] + HK42*P[6][19] + HK43*P[19][22] + HK44*P[19][23]);
            Kfusion[20] = HK54*(HK30*P[0][20] + HK35*P[1][20] + HK36*P[2][20] + HK37*P[3][20] + HK39*P[4][20] + HK41*P[5][20] + HK42*P[6][20] + HK43*P[20][22] + HK44*P[20][23]);
            Kfusion[21] = HK54*(HK30*P[0][21] + HK35*P[1][21] + HK36*P[2][21] + HK37*P[3][21] + HK39*P[4][21] + HK41*P[5][21] + HK42*P[6][21] + HK43*P[21][22] + HK44*P[21][23]);
        } else {
            // zero indexes 16 to 21 = 6*4 bytes
            memset(&Kfusion[16], 0, 24);
        }

        if (!inhibitWindStates) {
            Kfusion[22] = HK48*HK54;
            Kfusion[23] = HK50*HK54;
        } else {
            // zero indexes 22 to 23 = 2*4 bytes
            memset(&Kfusion[22], 0, 8);
        }

        // calculate predicted sideslip angle and innovation using small angle approximation
        innovBeta = vel_rel_wind.y / vel_rel_wind.x;

        // reject measurement if greater than 3-sigma inconsistency
        if (innovBeta > 0.5f) {
            return;
        }

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovBeta;
        }
        stateStruct.quat.normalize();

        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=6; j++) {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
            for (unsigned j = 7; j<=21; j++) {
                KH[i][j] = 0.0f;
            }
            for (unsigned j = 22; j<=23; j++) {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
        }
        for (unsigned j = 0; j<=stateIndexLim; j++) {
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                ftype res = 0;
                res += KH[i][0] * P[0][j];
                res += KH[i][1] * P[1][j];
                res += KH[i][2] * P[2][j];
                res += KH[i][3] * P[3][j];
                res += KH[i][4] * P[4][j];
                res += KH[i][5] * P[5][j];
                res += KH[i][6] * P[6][j];
                res += KH[i][22] * P[22][j];
                res += KH[i][23] * P[23][j];
                KHP[i][j] = res;
            }
        }
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop the performance timer
    hal.util->perf_end(_perf_FuseSideslip);
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

