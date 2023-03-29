#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_DAL/AP_DAL.h>

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

/*
 * Fuse true airspeed measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseAirspeed()
{
    // declarations
    ftype vn;
    ftype ve;
    ftype vd;
    ftype vwn;
    ftype vwe;
    ftype SH_TAS[3];
    ftype SK_TAS[2];
    Vector24 H_TAS = {};
    ftype VtasPred;

    // copy required states to local variable names
    vn = stateStruct.velocity.x;
    ve = stateStruct.velocity.y;
    vd = stateStruct.velocity.z;
    vwn = stateStruct.wind_vel.x;
    vwe = stateStruct.wind_vel.y;

    // calculate the predicted airspeed
    VtasPred = norm((ve - vwe) , (vn - vwn) , vd);
    // perform fusion of True Airspeed measurement
    if (VtasPred > 1.0f)
    {
        // calculate observation innovation and variance
        innovVtas = VtasPred - tasDataDelayed.tas;

        // calculate observation jacobians
        SH_TAS[0] = 1.0f/VtasPred;
        SH_TAS[1] = (SH_TAS[0]*(2.0f*ve - 2.0f*vwe))*0.5f;
        SH_TAS[2] = (SH_TAS[0]*(2.0f*vn - 2.0f*vwn))*0.5f;
        H_TAS[4] = SH_TAS[2];
        H_TAS[5] = SH_TAS[1];
        H_TAS[6] = vd*SH_TAS[0];
        H_TAS[22] = -SH_TAS[2];
        H_TAS[23] = -SH_TAS[1];
        // calculate Kalman gains
        ftype temp = (tasDataDelayed.tasVariance + SH_TAS[2]*(P[4][4]*SH_TAS[2] + P[5][4]*SH_TAS[1] - P[22][4]*SH_TAS[2] - P[23][4]*SH_TAS[1] + P[6][4]*vd*SH_TAS[0]) + SH_TAS[1]*(P[4][5]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[22][5]*SH_TAS[2] - P[23][5]*SH_TAS[1] + P[6][5]*vd*SH_TAS[0]) - SH_TAS[2]*(P[4][22]*SH_TAS[2] + P[5][22]*SH_TAS[1] - P[22][22]*SH_TAS[2] - P[23][22]*SH_TAS[1] + P[6][22]*vd*SH_TAS[0]) - SH_TAS[1]*(P[4][23]*SH_TAS[2] + P[5][23]*SH_TAS[1] - P[22][23]*SH_TAS[2] - P[23][23]*SH_TAS[1] + P[6][23]*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P[4][6]*SH_TAS[2] + P[5][6]*SH_TAS[1] - P[22][6]*SH_TAS[2] - P[23][6]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]));
        if (temp >= tasDataDelayed.tasVariance) {
            SK_TAS[0] = 1.0f / temp;
            faultStatus.bad_airspeed = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_airspeed = true;
            return;
        }
        SK_TAS[1] = SH_TAS[1];

        if (tasDataDelayed.allowFusion && !airDataFusionWindOnly) {
            Kfusion[0] = SK_TAS[0]*(P[0][4]*SH_TAS[2] - P[0][22]*SH_TAS[2] + P[0][5]*SK_TAS[1] - P[0][23]*SK_TAS[1] + P[0][6]*vd*SH_TAS[0]);
            Kfusion[1] = SK_TAS[0]*(P[1][4]*SH_TAS[2] - P[1][22]*SH_TAS[2] + P[1][5]*SK_TAS[1] - P[1][23]*SK_TAS[1] + P[1][6]*vd*SH_TAS[0]);
            Kfusion[2] = SK_TAS[0]*(P[2][4]*SH_TAS[2] - P[2][22]*SH_TAS[2] + P[2][5]*SK_TAS[1] - P[2][23]*SK_TAS[1] + P[2][6]*vd*SH_TAS[0]);
            Kfusion[3] = SK_TAS[0]*(P[3][4]*SH_TAS[2] - P[3][22]*SH_TAS[2] + P[3][5]*SK_TAS[1] - P[3][23]*SK_TAS[1] + P[3][6]*vd*SH_TAS[0]);
            Kfusion[4] = SK_TAS[0]*(P[4][4]*SH_TAS[2] - P[4][22]*SH_TAS[2] + P[4][5]*SK_TAS[1] - P[4][23]*SK_TAS[1] + P[4][6]*vd*SH_TAS[0]);
            Kfusion[5] = SK_TAS[0]*(P[5][4]*SH_TAS[2] - P[5][22]*SH_TAS[2] + P[5][5]*SK_TAS[1] - P[5][23]*SK_TAS[1] + P[5][6]*vd*SH_TAS[0]);
            Kfusion[6] = SK_TAS[0]*(P[6][4]*SH_TAS[2] - P[6][22]*SH_TAS[2] + P[6][5]*SK_TAS[1] - P[6][23]*SK_TAS[1] + P[6][6]*vd*SH_TAS[0]);
            Kfusion[7] = SK_TAS[0]*(P[7][4]*SH_TAS[2] - P[7][22]*SH_TAS[2] + P[7][5]*SK_TAS[1] - P[7][23]*SK_TAS[1] + P[7][6]*vd*SH_TAS[0]);
            Kfusion[8] = SK_TAS[0]*(P[8][4]*SH_TAS[2] - P[8][22]*SH_TAS[2] + P[8][5]*SK_TAS[1] - P[8][23]*SK_TAS[1] + P[8][6]*vd*SH_TAS[0]);
            Kfusion[9] = SK_TAS[0]*(P[9][4]*SH_TAS[2] - P[9][22]*SH_TAS[2] + P[9][5]*SK_TAS[1] - P[9][23]*SK_TAS[1] + P[9][6]*vd*SH_TAS[0]);
        } else {
            // zero indexes 0 to 9
            zero_range(&Kfusion[0], 0, 9);
        }

        if (tasDataDelayed.allowFusion && !inhibitDelAngBiasStates && !airDataFusionWindOnly) {
            Kfusion[10] = SK_TAS[0]*(P[10][4]*SH_TAS[2] - P[10][22]*SH_TAS[2] + P[10][5]*SK_TAS[1] - P[10][23]*SK_TAS[1] + P[10][6]*vd*SH_TAS[0]);
            Kfusion[11] = SK_TAS[0]*(P[11][4]*SH_TAS[2] - P[11][22]*SH_TAS[2] + P[11][5]*SK_TAS[1] - P[11][23]*SK_TAS[1] + P[11][6]*vd*SH_TAS[0]);
            Kfusion[12] = SK_TAS[0]*(P[12][4]*SH_TAS[2] - P[12][22]*SH_TAS[2] + P[12][5]*SK_TAS[1] - P[12][23]*SK_TAS[1] + P[12][6]*vd*SH_TAS[0]);
        } else {
            // zero indexes 10 to 12
            zero_range(&Kfusion[0], 10, 12);
        }

        if (tasDataDelayed.allowFusion && !inhibitDelVelBiasStates && !airDataFusionWindOnly) {
            for (uint8_t index = 0; index < 3; index++) {
                const uint8_t stateIndex = index + 13;
                if (!dvelBiasAxisInhibit[index]) {
                    Kfusion[stateIndex] = SK_TAS[0]*(P[stateIndex][4]*SH_TAS[2] - P[stateIndex][22]*SH_TAS[2] + P[stateIndex][5]*SK_TAS[1] - P[stateIndex][23]*SK_TAS[1] + P[stateIndex][6]*vd*SH_TAS[0]);
                } else {
                    Kfusion[stateIndex] = 0.0f;
                }
            }
        } else {
            // zero indexes 13 to 15
            zero_range(&Kfusion[0], 13, 15);
        }

        // zero Kalman gains to inhibit magnetic field state estimation
        if (tasDataDelayed.allowFusion && !inhibitMagStates && !airDataFusionWindOnly) {
            Kfusion[16] = SK_TAS[0]*(P[16][4]*SH_TAS[2] - P[16][22]*SH_TAS[2] + P[16][5]*SK_TAS[1] - P[16][23]*SK_TAS[1] + P[16][6]*vd*SH_TAS[0]);
            Kfusion[17] = SK_TAS[0]*(P[17][4]*SH_TAS[2] - P[17][22]*SH_TAS[2] + P[17][5]*SK_TAS[1] - P[17][23]*SK_TAS[1] + P[17][6]*vd*SH_TAS[0]);
            Kfusion[18] = SK_TAS[0]*(P[18][4]*SH_TAS[2] - P[18][22]*SH_TAS[2] + P[18][5]*SK_TAS[1] - P[18][23]*SK_TAS[1] + P[18][6]*vd*SH_TAS[0]);
            Kfusion[19] = SK_TAS[0]*(P[19][4]*SH_TAS[2] - P[19][22]*SH_TAS[2] + P[19][5]*SK_TAS[1] - P[19][23]*SK_TAS[1] + P[19][6]*vd*SH_TAS[0]);
            Kfusion[20] = SK_TAS[0]*(P[20][4]*SH_TAS[2] - P[20][22]*SH_TAS[2] + P[20][5]*SK_TAS[1] - P[20][23]*SK_TAS[1] + P[20][6]*vd*SH_TAS[0]);
            Kfusion[21] = SK_TAS[0]*(P[21][4]*SH_TAS[2] - P[21][22]*SH_TAS[2] + P[21][5]*SK_TAS[1] - P[21][23]*SK_TAS[1] + P[21][6]*vd*SH_TAS[0]);
        } else {
            // zero indexes 16 to 21
            zero_range(&Kfusion[0], 16, 21);
        }

        if (tasDataDelayed.allowFusion && !inhibitWindStates) {
            Kfusion[22] = SK_TAS[0]*(P[22][4]*SH_TAS[2] - P[22][22]*SH_TAS[2] + P[22][5]*SK_TAS[1] - P[22][23]*SK_TAS[1] + P[22][6]*vd*SH_TAS[0]);
            Kfusion[23] = SK_TAS[0]*(P[23][4]*SH_TAS[2] - P[23][22]*SH_TAS[2] + P[23][5]*SK_TAS[1] - P[23][23]*SK_TAS[1] + P[23][6]*vd*SH_TAS[0]);
        } else {
            // zero indexes 22 to 23 = 2
            zero_range(&Kfusion[0], 22, 23);
        }

        // calculate measurement innovation variance
        varInnovVtas = 1.0f/SK_TAS[0];

        // calculate the innovation consistency test ratio
        tasTestRatio = sq(innovVtas) / (sq(MAX(0.01f * (ftype)frontend->_tasInnovGate, 1.0f)) * varInnovVtas);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        const bool isConsistent = (tasTestRatio < 1.0f) || badIMUdata;
        tasTimeout = (imuSampleTime_ms - lastTasPassTime_ms) > frontend->tasRetryTime_ms;
        if (!isConsistent) {
            lastTasFailTime_ms = imuSampleTime_ms;
        } else {
            lastTasFailTime_ms = 0;
        }

        // test the ratio before fusing data, forcing fusion if airspeed and position are timed out as we have no choice but to try and use airspeed to constrain error growth
        if (tasDataDelayed.allowFusion && (isConsistent || (tasTimeout && posTimeout))) {

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
        // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
        ForceSymmetry();
        ConstrainVariances();
    }
}

void NavEKF3_core::FuseAirspeed2D()
{
    Vector24 Hfusion = {};

    // get latest estimated orientation
    const ftype &q0 = stateStruct.quat[0];
    const ftype &q1 = stateStruct.quat[1];
    const ftype &q2 = stateStruct.quat[2];
    const ftype &q3 = stateStruct.quat[3];

    // get latest velocity in earth frame
    const ftype &vn = stateStruct.velocity.x;
    const ftype &ve = stateStruct.velocity.y;
    const ftype &vd = stateStruct.velocity.z;

    // get latest wind velocity in earth frame
    const ftype &vwn = stateStruct.wind_vel.x;
    const ftype &vwe = stateStruct.wind_vel.y;

    const Vector3F rel_wind_earth(vn - vwn, ve - vwe, vd);
    const Vector3F rel_wind_body = prevTnb * rel_wind_earth;

    for (uint8_t axis_index=0; axis_index<2; axis_index++) {
        if (axis_index == 0) {

            innovVtas = rel_wind_body.x - tasDataDelayed.tas * cosF(radians(tasDataDelayed.direction));

            // Sub Expressions
            const ftype HK0 = q2*vd;
            const ftype HK1 = ve - vwe;
            const ftype HK2 = HK1*q3;
            const ftype HK3 = HK1*q2 + q3*vd;
            const ftype HK4 = q0*vd;
            const ftype HK5 = HK1*q1;
            const ftype HK6 = 2*vn - 2*vwn;
            const ftype HK7 = HK6*q2;
            const ftype HK8 = HK1*q0 - HK6*q3 + q1*vd;
            const ftype HK9 = 2*powf(q2, 2);
            const ftype HK10 = 2*powf(q3, 2);
            const ftype HK11 = q0*q3 + q1*q2;
            const ftype HK12 = 2*HK11;
            const ftype HK13 = q1*q3;
            const ftype HK14 = q0*q2;
            const ftype HK15 = HK10 + HK9 - 1;
            const ftype HK16 = 2*HK11;
            const ftype HK17 = -2*HK13 + 2*HK14;
            const ftype HK18 = 2*HK3;
            const ftype HK19 = 2*HK0 - 2*HK2;
            const ftype HK20 = 2*HK8;
            const ftype HK21 = 2*HK4 - 2*HK5 + 2*HK7;
            const ftype HK22 = -HK15*P[0][22] + HK15*P[0][4] + HK16*P[0][23] - HK16*P[0][5] + HK17*P[0][6] - HK18*P[0][1] + HK19*P[0][0] - HK20*P[0][3] + HK21*P[0][2];
            const ftype HK23 = HK16*P[5][23];
            const ftype HK24 = -HK15*P[22][23] + HK15*P[4][23] + HK16*P[23][23] + HK17*P[6][23] - HK18*P[1][23] + HK19*P[0][23] - HK20*P[3][23] + HK21*P[2][23] - HK23;
            const ftype HK25 = HK15*P[4][5] - HK15*P[5][22] - HK16*P[5][5] + HK17*P[5][6] - HK18*P[1][5] + HK19*P[0][5] - HK20*P[3][5] + HK21*P[2][5] + HK23;
            const ftype HK26 = HK15*P[4][6] - HK15*P[6][22] - HK16*P[5][6] + HK16*P[6][23] + HK17*P[6][6] - HK18*P[1][6] + HK19*P[0][6] - HK20*P[3][6] + HK21*P[2][6];
            const ftype HK27 = -HK15*P[1][22] + HK15*P[1][4] + HK16*P[1][23] - HK16*P[1][5] + HK17*P[1][6] - HK18*P[1][1] + HK19*P[0][1] - HK20*P[1][3] + HK21*P[1][2];
            const ftype HK28 = HK15*P[4][22];
            const ftype HK29 = HK15*P[4][4] + HK16*P[4][23] - HK16*P[4][5] + HK17*P[4][6] - HK18*P[1][4] + HK19*P[0][4] - HK20*P[3][4] + HK21*P[2][4] - HK28;
            const ftype HK30 = -HK15*P[22][22] + HK16*P[22][23] - HK16*P[5][22] + HK17*P[6][22] - HK18*P[1][22] + HK19*P[0][22] - HK20*P[3][22] + HK21*P[2][22] + HK28;
            const ftype HK31 = -HK15*P[3][22] + HK15*P[3][4] + HK16*P[3][23] - HK16*P[3][5] + HK17*P[3][6] - HK18*P[1][3] + HK19*P[0][3] - HK20*P[3][3] + HK21*P[2][3];
            const ftype HK32 = -HK15*P[2][22] + HK15*P[2][4] + HK16*P[2][23] - HK16*P[2][5] + HK17*P[2][6] - HK18*P[1][2] + HK19*P[0][2] - HK20*P[2][3] + HK21*P[2][2];
            varInnovVtas = (HK15*HK29 - HK15*HK30 + HK16*HK24 - HK16*HK25 + HK17*HK26 - HK18*HK27 + HK19*HK22 - HK20*HK31 + HK21*HK32 + tasDataDelayed.tasVariance);
            const ftype HK33 = 1.0F/HK29;

            // Observation Jacobians
            Hfusion[0] = -2*HK0 + 2*HK2;
            Hfusion[1] = 2*HK3;
            Hfusion[2] = -2*HK4 + 2*HK5 - 2*HK7;
            Hfusion[3] = 2*HK8;
            Hfusion[4] = -HK10 - HK9 + 1;
            Hfusion[5] = HK12;
            Hfusion[6] = 2*HK13 - 2*HK14;
            Hfusion[7] = 0;
            Hfusion[8] = 0;
            Hfusion[9] = 0;
            Hfusion[10] = 0;
            Hfusion[11] = 0;
            Hfusion[12] = 0;
            Hfusion[13] = 0;
            Hfusion[14] = 0;
            Hfusion[15] = 0;
            Hfusion[16] = 0;
            Hfusion[17] = 0;
            Hfusion[18] = 0;
            Hfusion[19] = 0;
            Hfusion[20] = 0;
            Hfusion[21] = 0;
            Hfusion[22] = HK15;
            Hfusion[23] = -HK12;

            // Kalman gains
            if (tasDataDelayed.allowFusion && !airDataFusionWindOnly) {
                Kfusion[0] = -HK22*HK33;
                Kfusion[1] = -HK27*HK33;
                Kfusion[2] = -HK32*HK33;
                Kfusion[3] = -HK31*HK33;
                Kfusion[4] = -HK29*HK33;
                Kfusion[5] = -HK25*HK33;
                Kfusion[6] = -HK26*HK33;
                Kfusion[7] = -HK33*(HK15*P[4][7] - HK15*P[7][22] - HK16*P[5][7] + HK16*P[7][23] + HK17*P[6][7] - HK18*P[1][7] + HK19*P[0][7] - HK20*P[3][7] + HK21*P[2][7]);
                Kfusion[8] = -HK33*(HK15*P[4][8] - HK15*P[8][22] - HK16*P[5][8] + HK16*P[8][23] + HK17*P[6][8] - HK18*P[1][8] + HK19*P[0][8] - HK20*P[3][8] + HK21*P[2][8]);
                Kfusion[9] = -HK33*(HK15*P[4][9] - HK15*P[9][22] - HK16*P[5][9] + HK16*P[9][23] + HK17*P[6][9] - HK18*P[1][9] + HK19*P[0][9] - HK20*P[3][9] + HK21*P[2][9]);
            } else {
                // zero indexes 0 to 9
                zero_range(&Kfusion[0], 0, 9);
            }

            if (tasDataDelayed.allowFusion && !inhibitDelAngBiasStates && !airDataFusionWindOnly) {
                Kfusion[10] = -HK33*(-HK15*P[10][22] + HK15*P[4][10] + HK16*P[10][23] - HK16*P[5][10] + HK17*P[6][10] - HK18*P[1][10] + HK19*P[0][10] - HK20*P[3][10] + HK21*P[2][10]);
                Kfusion[11] = -HK33*(-HK15*P[11][22] + HK15*P[4][11] + HK16*P[11][23] - HK16*P[5][11] + HK17*P[6][11] - HK18*P[1][11] + HK19*P[0][11] - HK20*P[3][11] + HK21*P[2][11]);
                Kfusion[12] = -HK33*(-HK15*P[12][22] + HK15*P[4][12] + HK16*P[12][23] - HK16*P[5][12] + HK17*P[6][12] - HK18*P[1][12] + HK19*P[0][12] - HK20*P[3][12] + HK21*P[2][12]);
            } else {
                // zero indexes 10 to 12
                zero_range(&Kfusion[0], 10, 12);
            }

            if (tasDataDelayed.allowFusion && !inhibitDelVelBiasStates && !airDataFusionWindOnly) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!dvelBiasAxisInhibit[index]) {
                        Kfusion[stateIndex] = -HK33*(-HK15*P[stateIndex][22] + HK15*P[4][stateIndex] + HK16*P[stateIndex][23] - HK16*P[5][stateIndex] + HK17*P[6][stateIndex] - HK18*P[1][stateIndex] + HK19*P[0][stateIndex] - HK20*P[3][stateIndex] + HK21*P[2][stateIndex]);
                    } else {
                        Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                zero_range(&Kfusion[0], 13, 15);
            }

            // zero Kalman gains to inhibit magnetic field state estimation
            if (tasDataDelayed.allowFusion && !inhibitMagStates && !airDataFusionWindOnly) {
                Kfusion[16] = -HK33*(-HK15*P[16][22] + HK15*P[4][16] + HK16*P[16][23] - HK16*P[5][16] + HK17*P[6][16] - HK18*P[1][16] + HK19*P[0][16] - HK20*P[3][16] + HK21*P[2][16]);
                Kfusion[17] = -HK33*(-HK15*P[17][22] + HK15*P[4][17] + HK16*P[17][23] - HK16*P[5][17] + HK17*P[6][17] - HK18*P[1][17] + HK19*P[0][17] - HK20*P[3][17] + HK21*P[2][17]);
                Kfusion[18] = -HK33*(-HK15*P[18][22] + HK15*P[4][18] + HK16*P[18][23] - HK16*P[5][18] + HK17*P[6][18] - HK18*P[1][18] + HK19*P[0][18] - HK20*P[3][18] + HK21*P[2][18]);
                Kfusion[19] = -HK33*(-HK15*P[19][22] + HK15*P[4][19] + HK16*P[19][23] - HK16*P[5][19] + HK17*P[6][19] - HK18*P[1][19] + HK19*P[0][19] - HK20*P[3][19] + HK21*P[2][19]);
                Kfusion[20] = -HK33*(-HK15*P[20][22] + HK15*P[4][20] + HK16*P[20][23] - HK16*P[5][20] + HK17*P[6][20] - HK18*P[1][20] + HK19*P[0][20] - HK20*P[3][20] + HK21*P[2][20]);
                Kfusion[21] = -HK33*(-HK15*P[21][22] + HK15*P[4][21] + HK16*P[21][23] - HK16*P[5][21] + HK17*P[6][21] - HK18*P[1][21] + HK19*P[0][21] - HK20*P[3][21] + HK21*P[2][21]);
            } else {
                // zero indexes 16 to 21
                zero_range(&Kfusion[0], 16, 21);
            }

            if (tasDataDelayed.allowFusion && !inhibitWindStates) {
                Kfusion[22] = -HK30*HK33;
                Kfusion[23] = -HK24*HK33;
            } else {
                // zero indexes 22 to 23 = 2
                zero_range(&Kfusion[0], 22, 23);
            }

        } else if (axis_index == 1) {
            innovVtas = rel_wind_body.x - tasDataDelayed.tas * sinF(radians(tasDataDelayed.direction));

            // Sub Expressions
            const ftype HK0 = vn - vwn;
            const ftype HK1 = -HK0*q3 + q1*vd;
            const ftype HK2 = 2*ve - 2*vwe;
            const ftype HK3 = HK0*q2 - HK2*q1 + q0*vd;
            const ftype HK4 = HK0*q1 + q3*vd;
            const ftype HK5 = q2*vd;
            const ftype HK6 = HK0*q0;
            const ftype HK7 = HK2*q3;
            const ftype HK8 = q1*q2;
            const ftype HK9 = q0*q3;
            const ftype HK10 = 2*powf(q1, 2);
            const ftype HK11 = 2*powf(q3, 2);
            const ftype HK12 = q0*q1 + q2*q3;
            const ftype HK13 = -HK8 + HK9;
            const ftype HK14 = HK10 + HK11 - 1;
            const ftype HK15 = 2*HK12;
            const ftype HK16 = 2*HK13;
            const ftype HK17 = 2*HK4;
            const ftype HK18 = 2*HK1;
            const ftype HK19 = 2*HK3;
            const ftype HK20 = -2*HK5 + 2*HK6 + 2*HK7;
            const ftype HK21 = HK14*P[0][23] - HK14*P[0][5] + HK15*P[0][6] + HK16*P[0][22] - HK16*P[0][4] + HK17*P[0][2] + HK18*P[0][0] + HK19*P[0][1] - HK20*P[0][3];
            const ftype HK22 = -HK14*P[5][6] + HK14*P[6][23] + HK15*P[6][6] - HK16*P[4][6] + HK16*P[6][22] + HK17*P[2][6] + HK18*P[0][6] + HK19*P[1][6] - HK20*P[3][6];
            const ftype HK23 = HK16*P[4][22];
            const ftype HK24 = HK14*P[22][23] - HK14*P[5][22] + HK15*P[6][22] + HK16*P[22][22] + HK17*P[2][22] + HK18*P[0][22] + HK19*P[1][22] - HK20*P[3][22] - HK23;
            const ftype HK25 = HK14*P[4][23] - HK14*P[4][5] + HK15*P[4][6] - HK16*P[4][4] + HK17*P[2][4] + HK18*P[0][4] + HK19*P[1][4] - HK20*P[3][4] + HK23;
            const ftype HK26 = HK14*P[2][23] - HK14*P[2][5] + HK15*P[2][6] + HK16*P[2][22] - HK16*P[2][4] + HK17*P[2][2] + HK18*P[0][2] + HK19*P[1][2] - HK20*P[2][3];
            const ftype HK27 = HK14*P[5][23];
            const ftype HK28 = HK14*P[23][23] + HK15*P[6][23] + HK16*P[22][23] - HK16*P[4][23] + HK17*P[2][23] + HK18*P[0][23] + HK19*P[1][23] - HK20*P[3][23] - HK27;
            const ftype HK29 = -HK14*P[5][5] + HK15*P[5][6] - HK16*P[4][5] + HK16*P[5][22] + HK17*P[2][5] + HK18*P[0][5] + HK19*P[1][5] - HK20*P[3][5] + HK27;
            const ftype HK30 = HK14*P[1][23] - HK14*P[1][5] + HK15*P[1][6] + HK16*P[1][22] - HK16*P[1][4] + HK17*P[1][2] + HK18*P[0][1] + HK19*P[1][1] - HK20*P[1][3];
            const ftype HK31 = HK14*P[3][23] - HK14*P[3][5] + HK15*P[3][6] + HK16*P[3][22] - HK16*P[3][4] + HK17*P[2][3] + HK18*P[0][3] + HK19*P[1][3] - HK20*P[3][3];
            varInnovVtas = (HK14*HK28 - HK14*HK29 + HK15*HK22 + HK16*HK24 - HK16*HK25 + HK17*HK26 + HK18*HK21 + HK19*HK30 - HK20*HK31 + tasDataDelayed.tasVariance);
            const ftype HK32 = 1.0F/varInnovVtas;

            // Observation Jacobians
            Hfusion[0] = 2*HK1;
            Hfusion[1] = 2*HK3;
            Hfusion[2] = 2*HK4;
            Hfusion[3] = 2*HK5 - 2*HK6 - 2*HK7;
            Hfusion[4] = 2*HK8 - 2*HK9;
            Hfusion[5] = -HK10 - HK11 + 1;
            Hfusion[6] = 2*HK12;
            Hfusion[7] = 0;
            Hfusion[8] = 0;
            Hfusion[9] = 0;
            Hfusion[10] = 0;
            Hfusion[11] = 0;
            Hfusion[12] = 0;
            Hfusion[13] = 0;
            Hfusion[14] = 0;
            Hfusion[15] = 0;
            Hfusion[16] = 0;
            Hfusion[17] = 0;
            Hfusion[18] = 0;
            Hfusion[19] = 0;
            Hfusion[20] = 0;
            Hfusion[21] = 0;

            // Kalman gains
            if (tasDataDelayed.allowFusion && !airDataFusionWindOnly) {
                Kfusion[0] = HK21*HK32;
                Kfusion[1] = HK30*HK32;
                Kfusion[2] = HK26*HK32;
                Kfusion[3] = HK31*HK32;
                Kfusion[4] = HK25*HK32;
                Kfusion[5] = HK29*HK32;
                Kfusion[6] = HK22*HK32;
                Kfusion[7] = HK32*(-HK14*P[5][7] + HK14*P[7][23] + HK15*P[6][7] - HK16*P[4][7] + HK16*P[7][22] + HK17*P[2][7] + HK18*P[0][7] + HK19*P[1][7] - HK20*P[3][7]);
                Kfusion[8] = HK32*(-HK14*P[5][8] + HK14*P[8][23] + HK15*P[6][8] - HK16*P[4][8] + HK16*P[8][22] + HK17*P[2][8] + HK18*P[0][8] + HK19*P[1][8] - HK20*P[3][8]);
                Kfusion[9] = HK32*(-HK14*P[5][9] + HK14*P[9][23] + HK15*P[6][9] - HK16*P[4][9] + HK16*P[9][22] + HK17*P[2][9] + HK18*P[0][9] + HK19*P[1][9] - HK20*P[3][9]);
            } else {
                // zero indexes 0 to 9
                zero_range(&Kfusion[0], 0, 9);
            }

            if (tasDataDelayed.allowFusion && !inhibitDelAngBiasStates && !airDataFusionWindOnly) {
                Kfusion[10] = HK32*(HK14*P[10][23] - HK14*P[5][10] + HK15*P[6][10] + HK16*P[10][22] - HK16*P[4][10] + HK17*P[2][10] + HK18*P[0][10] + HK19*P[1][10] - HK20*P[3][10]);
                Kfusion[11] = HK32*(HK14*P[11][23] - HK14*P[5][11] + HK15*P[6][11] + HK16*P[11][22] - HK16*P[4][11] + HK17*P[2][11] + HK18*P[0][11] + HK19*P[1][11] - HK20*P[3][11]);
                Kfusion[12] = HK32*(HK14*P[12][23] - HK14*P[5][12] + HK15*P[6][12] + HK16*P[12][22] - HK16*P[4][12] + HK17*P[2][12] + HK18*P[0][12] + HK19*P[1][12] - HK20*P[3][12]);
            } else {
                // zero indexes 10 to 12
                zero_range(&Kfusion[0], 10, 12);
            }

            if (tasDataDelayed.allowFusion && !inhibitDelVelBiasStates && !airDataFusionWindOnly) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!dvelBiasAxisInhibit[index]) {
                        Kfusion[stateIndex] = HK32*(HK14*P[stateIndex][23] - HK14*P[5][stateIndex] + HK15*P[6][stateIndex] + HK16*P[stateIndex][22] - HK16*P[4][stateIndex] + HK17*P[2][stateIndex] + HK18*P[0][stateIndex] + HK19*P[1][stateIndex] - HK20*P[3][stateIndex]);
                    } else {
                        Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                zero_range(&Kfusion[0], 13, 15);
            }

            // zero Kalman gains to inhibit magnetic field state estimation
            if (tasDataDelayed.allowFusion && !inhibitMagStates && !airDataFusionWindOnly) {
                Kfusion[16] = HK32*(HK14*P[16][23] - HK14*P[5][16] + HK15*P[6][16] + HK16*P[16][22] - HK16*P[4][16] + HK17*P[2][16] + HK18*P[0][16] + HK19*P[1][16] - HK20*P[3][16]);
                Kfusion[17] = HK32*(HK14*P[17][23] - HK14*P[5][17] + HK15*P[6][17] + HK16*P[17][22] - HK16*P[4][17] + HK17*P[2][17] + HK18*P[0][17] + HK19*P[1][17] - HK20*P[3][17]);
                Kfusion[18] = HK32*(HK14*P[18][23] - HK14*P[5][18] + HK15*P[6][18] + HK16*P[18][22] - HK16*P[4][18] + HK17*P[2][18] + HK18*P[0][18] + HK19*P[1][18] - HK20*P[3][18]);
                Kfusion[19] = HK32*(HK14*P[19][23] - HK14*P[5][19] + HK15*P[6][19] + HK16*P[19][22] - HK16*P[4][19] + HK17*P[2][19] + HK18*P[0][19] + HK19*P[1][19] - HK20*P[3][19]);
                Kfusion[20] = HK32*(HK14*P[20][23] - HK14*P[5][20] + HK15*P[6][20] + HK16*P[20][22] - HK16*P[4][20] + HK17*P[2][20] + HK18*P[0][20] + HK19*P[1][20] - HK20*P[3][20]);
                Kfusion[21] = HK32*(HK14*P[21][23] - HK14*P[5][21] + HK15*P[6][21] + HK16*P[21][22] - HK16*P[4][21] + HK17*P[2][21] + HK18*P[0][21] + HK19*P[1][21] - HK20*P[3][21]);
            } else {
                // zero indexes 16 to 21
                zero_range(&Kfusion[0], 16, 21);
            }

            if (tasDataDelayed.allowFusion && !inhibitWindStates) {
                Kfusion[22] = HK24*HK32;
                Kfusion[23] = HK28*HK32;
            } else {
                // zero indexes 22 to 23 = 2
                zero_range(&Kfusion[0], 22, 23);
            }

        }

        // calculate the innovation consistency test ratio
        tasTestRatio = sq(innovVtas) / (sq(MAX(0.01f * (ftype)frontend->_tasInnovGate, 1.0f)) * varInnovVtas);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        const bool isConsistent = (tasTestRatio < 1.0f) || badIMUdata;
        tasTimeout = (imuSampleTime_ms - lastTasPassTime_ms) > frontend->tasRetryTime_ms;
        if (!isConsistent) {
            lastTasFailTime_ms = imuSampleTime_ms;
        } else {
            lastTasFailTime_ms = 0;
        }

        if (tasDataDelayed.allowFusion && (isConsistent || (tasTimeout && posTimeout))) {

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
                for (unsigned j = 0; j<=6; j++) {
                    KH[i][j] = Kfusion[i] * Hfusion[j];
                }
                for (unsigned j = 7; j<=21; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 22; j<=23; j++) {
                    KH[i][j] = Kfusion[i] * Hfusion[j];
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

    }
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

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then perform TAS fusion
    if (tasDataToFuse && statesInitialised && !inhibitWindStates) {
        if (tasDataDelayed.direction < 0.0f) {
            FuseAirspeed();
        } else {
            FuseAirspeed2D();
        }
        prevTasStep_ms = imuSampleTime_ms;
    }
}


// select fusion of synthetic sideslip measurements or body frame drag
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// body frame drag only works for bluff body multi rotor vehices with thrust forces aligned with the Z axis
// it requires a stable wind for best results and should not be used for aerobatic flight
void NavEKF3_core::SelectBetaDragFusion()
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
    bool f_timeTrigger = ((imuSampleTime_ms - prevBetaDragStep_ms) >= frontend->betaAvg_ms);

    // use of air data to constrain drift is necessary if we have limited sensor data or are doing inertial dead reckoning
    bool is_dead_reckoning = ((imuSampleTime_ms - lastPosPassTime_ms) > frontend->deadReckonDeclare_ms) && ((imuSampleTime_ms - lastVelPassTime_ms) > frontend->deadReckonDeclare_ms);
    const bool noYawSensor = !use_compass() && !using_noncompass_for_yaw();
    const bool f_required = (noYawSensor && (frontend->_betaMask & (1<<1))) || is_dead_reckoning;

    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    const bool f_beta_feasible = (assume_zero_sideslip() && !inhibitWindStates);

    // use synthetic sideslip fusion if feasible, required and enough time has lapsed since the last fusion
    if (f_beta_feasible && f_timeTrigger) {
        // unless air data is required to constrain drift, it is only used to update wind state estimates
        if (f_required || (frontend->_betaMask & (1<<0))) {
            // we are required to correct all states
            airDataFusionWindOnly = false;
        } else {
            // we are required to correct only wind states
            airDataFusionWindOnly = true;
        }
        // Fuse estimated airspeed to aid wind estimation
        if (usingDefaultAirspeed) {
            FuseAirspeed();
        }
        FuseSideslip();
        prevBetaDragStep_ms = imuSampleTime_ms;
    }

#if EK3_FEATURE_DRAG_FUSION
    // fusion of XY body frame aero specific forces is done at a slower rate and only if alternative methods of wind estimation are not available
    if (!inhibitWindStates && storedDrag.recall(dragSampleDelayed,imuDataDelayed.time_ms)) {
        FuseDragForces();
    }
    dragTimeout = (imuSampleTime_ms - lastDragPassTime_ms) > frontend->dragFailTimeLimit_ms;
#endif
}

/*
 * Fuse sythetic sideslip measurement of zero using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseSideslip()
{
    // declarations
    ftype q0;
    ftype q1;
    ftype q2;
    ftype q3;
    ftype vn;
    ftype ve;
    ftype vd;
    ftype vwn;
    ftype vwe;
    const ftype R_BETA = 0.03f; // assume a sideslip angle RMS of ~10 deg
    Vector13 SH_BETA;
    Vector8 SK_BETA;
    Vector3F vel_rel_wind;
    Vector24 H_BETA;

    // copy required states to local variable names
    q0 = stateStruct.quat[0];
    q1 = stateStruct.quat[1];
    q2 = stateStruct.quat[2];
    q3 = stateStruct.quat[3];
    vn = stateStruct.velocity.x;
    ve = stateStruct.velocity.y;
    vd = stateStruct.velocity.z;
    vwn = stateStruct.wind_vel.x;
    vwe = stateStruct.wind_vel.y;

    // calculate predicted wind relative velocity in NED
    vel_rel_wind.x = vn - vwn;
    vel_rel_wind.y = ve - vwe;
    vel_rel_wind.z = vd;

    // rotate into body axes
    vel_rel_wind = prevTnb * vel_rel_wind;

    // perform fusion of assumed sideslip  = 0
    if (vel_rel_wind.x > 5.0f)
    {
        // Calculate observation jacobians
        SH_BETA[0] = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + (ve - vwe)*(2*q0*q3 + 2*q1*q2);
        if (fabsF(SH_BETA[0]) <= 1e-9f) {
            faultStatus.bad_sideslip = true;
            return;
        } else {
            faultStatus.bad_sideslip = false;
        }
        SH_BETA[1] = (ve - vwe)*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - (vn - vwn)*(2*q0*q3 - 2*q1*q2);
        SH_BETA[2] = vn - vwn;
        SH_BETA[3] = ve - vwe;
        SH_BETA[4] = 1/sq(SH_BETA[0]);
        SH_BETA[5] = 1/SH_BETA[0];
        SH_BETA[6] = SH_BETA[5]*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
        SH_BETA[7] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_BETA[8] = 2*q0*SH_BETA[3] - 2*q3*SH_BETA[2] + 2*q1*vd;
        SH_BETA[9] = 2*q0*SH_BETA[2] + 2*q3*SH_BETA[3] - 2*q2*vd;
        SH_BETA[10] = 2*q2*SH_BETA[2] - 2*q1*SH_BETA[3] + 2*q0*vd;
        SH_BETA[11] = 2*q1*SH_BETA[2] + 2*q2*SH_BETA[3] + 2*q3*vd;
        SH_BETA[12] = 2*q0*q3;

        H_BETA[0] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
        H_BETA[1] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
        H_BETA[2] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
        H_BETA[3] = - SH_BETA[5]*SH_BETA[9] - SH_BETA[1]*SH_BETA[4]*SH_BETA[8];
        H_BETA[4] = - SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) - SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        H_BETA[5] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2);
        H_BETA[6] = SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3);
        for (uint8_t i=7; i<=21; i++) {
            H_BETA[i] = 0.0f;
        }
        H_BETA[22] = SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        H_BETA[23] = SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2) - SH_BETA[6];

        // Calculate Kalman gains
        ftype temp = (R_BETA - (SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P[22][4]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][4]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][4]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][4]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][4]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][4]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][4]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][4]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][4]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P[22][22]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][22]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][22]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][22]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][22]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][22]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][22]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][22]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][22]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2))*(P[22][5]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][5]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][5]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][5]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][5]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][5]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][5]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][5]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][5]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) - (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2))*(P[22][23]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][23]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][23]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][23]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][23]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][23]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][23]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][23]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][23]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9])*(P[22][0]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][0]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][0]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][0]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][0]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][0]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][0]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][0]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][0]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11])*(P[22][1]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][1]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][1]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][1]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][1]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][1]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][1]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][1]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][1]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10])*(P[22][2]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][2]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][2]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][2]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][2]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][2]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][2]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][2]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][2]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) - (SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8])*(P[22][3]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][3]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][3]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][3]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][3]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][3]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][3]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][3]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][3]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))*(P[22][6]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][6]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][6]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[23][6]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][6]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][6]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][6]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][6]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][6]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))));
        if (temp >= R_BETA) {
            SK_BETA[0] = 1.0f / temp;
            faultStatus.bad_sideslip = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_sideslip = true;
            return;
        }
        SK_BETA[1] = SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        SK_BETA[2] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2);
        SK_BETA[3] = SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3);
        SK_BETA[4] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
        SK_BETA[5] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
        SK_BETA[6] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
        SK_BETA[7] = SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8];

        if (!airDataFusionWindOnly) {
            Kfusion[0] = SK_BETA[0]*(P[0][0]*SK_BETA[5] + P[0][1]*SK_BETA[4] - P[0][4]*SK_BETA[1] + P[0][5]*SK_BETA[2] + P[0][2]*SK_BETA[6] + P[0][6]*SK_BETA[3] - P[0][3]*SK_BETA[7] + P[0][22]*SK_BETA[1] - P[0][23]*SK_BETA[2]);
            Kfusion[1] = SK_BETA[0]*(P[1][0]*SK_BETA[5] + P[1][1]*SK_BETA[4] - P[1][4]*SK_BETA[1] + P[1][5]*SK_BETA[2] + P[1][2]*SK_BETA[6] + P[1][6]*SK_BETA[3] - P[1][3]*SK_BETA[7] + P[1][22]*SK_BETA[1] - P[1][23]*SK_BETA[2]);
            Kfusion[2] = SK_BETA[0]*(P[2][0]*SK_BETA[5] + P[2][1]*SK_BETA[4] - P[2][4]*SK_BETA[1] + P[2][5]*SK_BETA[2] + P[2][2]*SK_BETA[6] + P[2][6]*SK_BETA[3] - P[2][3]*SK_BETA[7] + P[2][22]*SK_BETA[1] - P[2][23]*SK_BETA[2]);
            Kfusion[3] = SK_BETA[0]*(P[3][0]*SK_BETA[5] + P[3][1]*SK_BETA[4] - P[3][4]*SK_BETA[1] + P[3][5]*SK_BETA[2] + P[3][2]*SK_BETA[6] + P[3][6]*SK_BETA[3] - P[3][3]*SK_BETA[7] + P[3][22]*SK_BETA[1] - P[3][23]*SK_BETA[2]);
            Kfusion[4] = SK_BETA[0]*(P[4][0]*SK_BETA[5] + P[4][1]*SK_BETA[4] - P[4][4]*SK_BETA[1] + P[4][5]*SK_BETA[2] + P[4][2]*SK_BETA[6] + P[4][6]*SK_BETA[3] - P[4][3]*SK_BETA[7] + P[4][22]*SK_BETA[1] - P[4][23]*SK_BETA[2]);
            Kfusion[5] = SK_BETA[0]*(P[5][0]*SK_BETA[5] + P[5][1]*SK_BETA[4] - P[5][4]*SK_BETA[1] + P[5][5]*SK_BETA[2] + P[5][2]*SK_BETA[6] + P[5][6]*SK_BETA[3] - P[5][3]*SK_BETA[7] + P[5][22]*SK_BETA[1] - P[5][23]*SK_BETA[2]);
            Kfusion[6] = SK_BETA[0]*(P[6][0]*SK_BETA[5] + P[6][1]*SK_BETA[4] - P[6][4]*SK_BETA[1] + P[6][5]*SK_BETA[2] + P[6][2]*SK_BETA[6] + P[6][6]*SK_BETA[3] - P[6][3]*SK_BETA[7] + P[6][22]*SK_BETA[1] - P[6][23]*SK_BETA[2]);
            Kfusion[7] = SK_BETA[0]*(P[7][0]*SK_BETA[5] + P[7][1]*SK_BETA[4] - P[7][4]*SK_BETA[1] + P[7][5]*SK_BETA[2] + P[7][2]*SK_BETA[6] + P[7][6]*SK_BETA[3] - P[7][3]*SK_BETA[7] + P[7][22]*SK_BETA[1] - P[7][23]*SK_BETA[2]);
            Kfusion[8] = SK_BETA[0]*(P[8][0]*SK_BETA[5] + P[8][1]*SK_BETA[4] - P[8][4]*SK_BETA[1] + P[8][5]*SK_BETA[2] + P[8][2]*SK_BETA[6] + P[8][6]*SK_BETA[3] - P[8][3]*SK_BETA[7] + P[8][22]*SK_BETA[1] - P[8][23]*SK_BETA[2]);
            Kfusion[9] = SK_BETA[0]*(P[9][0]*SK_BETA[5] + P[9][1]*SK_BETA[4] - P[9][4]*SK_BETA[1] + P[9][5]*SK_BETA[2] + P[9][2]*SK_BETA[6] + P[9][6]*SK_BETA[3] - P[9][3]*SK_BETA[7] + P[9][22]*SK_BETA[1] - P[9][23]*SK_BETA[2]);
        } else {
            // zero indexes 0 to 9
            zero_range(&Kfusion[0], 0, 9);
        }

        if (!inhibitDelAngBiasStates && !airDataFusionWindOnly) {
            Kfusion[10] = SK_BETA[0]*(P[10][0]*SK_BETA[5] + P[10][1]*SK_BETA[4] - P[10][4]*SK_BETA[1] + P[10][5]*SK_BETA[2] + P[10][2]*SK_BETA[6] + P[10][6]*SK_BETA[3] - P[10][3]*SK_BETA[7] + P[10][22]*SK_BETA[1] - P[10][23]*SK_BETA[2]);
            Kfusion[11] = SK_BETA[0]*(P[11][0]*SK_BETA[5] + P[11][1]*SK_BETA[4] - P[11][4]*SK_BETA[1] + P[11][5]*SK_BETA[2] + P[11][2]*SK_BETA[6] + P[11][6]*SK_BETA[3] - P[11][3]*SK_BETA[7] + P[11][22]*SK_BETA[1] - P[11][23]*SK_BETA[2]);
            Kfusion[12] = SK_BETA[0]*(P[12][0]*SK_BETA[5] + P[12][1]*SK_BETA[4] - P[12][4]*SK_BETA[1] + P[12][5]*SK_BETA[2] + P[12][2]*SK_BETA[6] + P[12][6]*SK_BETA[3] - P[12][3]*SK_BETA[7] + P[12][22]*SK_BETA[1] - P[12][23]*SK_BETA[2]);
        } else {
            // zero indexes 10 to 12 = 3
            zero_range(&Kfusion[0], 10, 12);
        }

        if (!inhibitDelVelBiasStates && !airDataFusionWindOnly) {
            for (uint8_t index = 0; index < 3; index++) {
                const uint8_t stateIndex = index + 13;
                if (!dvelBiasAxisInhibit[index]) {
                    Kfusion[stateIndex] = SK_BETA[0]*(P[stateIndex][0]*SK_BETA[5] + P[stateIndex][1]*SK_BETA[4] - P[stateIndex][4]*SK_BETA[1] + P[stateIndex][5]*SK_BETA[2] + P[stateIndex][2]*SK_BETA[6] + P[stateIndex][6]*SK_BETA[3] - P[stateIndex][3]*SK_BETA[7] + P[stateIndex][22]*SK_BETA[1] - P[stateIndex][23]*SK_BETA[2]);
                } else {
                    Kfusion[stateIndex] = 0.0f;
                }
            }
        } else {
            // zero indexes 13 to 15
            zero_range(&Kfusion[0], 13, 15);
        }

        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates && !airDataFusionWindOnly) {
            Kfusion[16] = SK_BETA[0]*(P[16][0]*SK_BETA[5] + P[16][1]*SK_BETA[4] - P[16][4]*SK_BETA[1] + P[16][5]*SK_BETA[2] + P[16][2]*SK_BETA[6] + P[16][6]*SK_BETA[3] - P[16][3]*SK_BETA[7] + P[16][22]*SK_BETA[1] - P[16][23]*SK_BETA[2]);
            Kfusion[17] = SK_BETA[0]*(P[17][0]*SK_BETA[5] + P[17][1]*SK_BETA[4] - P[17][4]*SK_BETA[1] + P[17][5]*SK_BETA[2] + P[17][2]*SK_BETA[6] + P[17][6]*SK_BETA[3] - P[17][3]*SK_BETA[7] + P[17][22]*SK_BETA[1] - P[17][23]*SK_BETA[2]);
            Kfusion[18] = SK_BETA[0]*(P[18][0]*SK_BETA[5] + P[18][1]*SK_BETA[4] - P[18][4]*SK_BETA[1] + P[18][5]*SK_BETA[2] + P[18][2]*SK_BETA[6] + P[18][6]*SK_BETA[3] - P[18][3]*SK_BETA[7] + P[18][22]*SK_BETA[1] - P[18][23]*SK_BETA[2]);
            Kfusion[19] = SK_BETA[0]*(P[19][0]*SK_BETA[5] + P[19][1]*SK_BETA[4] - P[19][4]*SK_BETA[1] + P[19][5]*SK_BETA[2] + P[19][2]*SK_BETA[6] + P[19][6]*SK_BETA[3] - P[19][3]*SK_BETA[7] + P[19][22]*SK_BETA[1] - P[19][23]*SK_BETA[2]);
            Kfusion[20] = SK_BETA[0]*(P[20][0]*SK_BETA[5] + P[20][1]*SK_BETA[4] - P[20][4]*SK_BETA[1] + P[20][5]*SK_BETA[2] + P[20][2]*SK_BETA[6] + P[20][6]*SK_BETA[3] - P[20][3]*SK_BETA[7] + P[20][22]*SK_BETA[1] - P[20][23]*SK_BETA[2]);
            Kfusion[21] = SK_BETA[0]*(P[21][0]*SK_BETA[5] + P[21][1]*SK_BETA[4] - P[21][4]*SK_BETA[1] + P[21][5]*SK_BETA[2] + P[21][2]*SK_BETA[6] + P[21][6]*SK_BETA[3] - P[21][3]*SK_BETA[7] + P[21][22]*SK_BETA[1] - P[21][23]*SK_BETA[2]);
        } else {
            // zero indexes 16 to 21
            zero_range(&Kfusion[0], 16, 21);
        }

        if (!inhibitWindStates) {
            Kfusion[22] = SK_BETA[0]*(P[22][0]*SK_BETA[5] + P[22][1]*SK_BETA[4] - P[22][4]*SK_BETA[1] + P[22][5]*SK_BETA[2] + P[22][2]*SK_BETA[6] + P[22][6]*SK_BETA[3] - P[22][3]*SK_BETA[7] + P[22][22]*SK_BETA[1] - P[22][23]*SK_BETA[2]);
            Kfusion[23] = SK_BETA[0]*(P[23][0]*SK_BETA[5] + P[23][1]*SK_BETA[4] - P[23][4]*SK_BETA[1] + P[23][5]*SK_BETA[2] + P[23][2]*SK_BETA[6] + P[23][6]*SK_BETA[3] - P[23][3]*SK_BETA[7] + P[23][22]*SK_BETA[1] - P[23][23]*SK_BETA[2]);
        } else {
            // zero indexes 22 to 23
            zero_range(&Kfusion[0], 22, 23);
        }

        // calculate predicted sideslip angle and innovation using small angle approximation
        innovBeta = constrain_ftype(vel_rel_wind.y / vel_rel_wind.x, -0.5f, 0.5f);

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
}

#if EK3_FEATURE_DRAG_FUSION
/*
 * Fuse X and Y body axis specific forces using explicit algebraic equations generated with SymPy.
 * See AP_NavEKF3/derivation/main.py for derivation
 * Output for change reference: AP_NavEKF3/derivation/generated/acc_bf_generated.cpp
*/
void NavEKF3_core::FuseDragForces()
{
    // drag model parameters
    const ftype bcoef_x = frontend->_ballisticCoef_x;
    const ftype bcoef_y = frontend->_ballisticCoef_y;
    const ftype mcoef = frontend->_momentumDragCoef.get();
    const bool using_bcoef_x = bcoef_x > 1.0f;
    const bool using_bcoef_y = bcoef_y > 1.0f;
    const bool using_mcoef = mcoef > 0.001f;

    ZERO_FARRAY(Kfusion);
    Vector24 Hfusion; // Observation Jacobians
    const ftype R_ACC = sq(fmaxF(frontend->_dragObsNoise, 0.5f));
    const ftype density_ratio = sqrtF(dal.get_EAS2TAS());
    const ftype rho = fmaxF(1.225f * density_ratio, 0.1f); // air density

    // get latest estimated orientation
    const ftype &q0 = stateStruct.quat[0];
    const ftype &q1 = stateStruct.quat[1];
    const ftype &q2 = stateStruct.quat[2];
    const ftype &q3 = stateStruct.quat[3];

    // get latest velocity in earth frame
    const ftype &vn = stateStruct.velocity.x;
    const ftype &ve = stateStruct.velocity.y;
    const ftype &vd = stateStruct.velocity.z;

    // get latest wind velocity in earth frame
    const ftype &vwn = stateStruct.wind_vel.x;
    const ftype &vwe = stateStruct.wind_vel.y;

    // predicted specific forces
    // calculate relative wind velocity in earth frame and rotate into body frame
    const Vector3F rel_wind_earth(vn - vwn, ve - vwe, vd);
    const Vector3F rel_wind_body = prevTnb * rel_wind_earth;

    // perform sequential fusion of XY specific forces
    for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
        // correct accel data for bias
        const ftype mea_acc = dragSampleDelayed.accelXY[axis_index]  - stateStruct.accel_bias[axis_index] / dtEkfAvg;

        // Acceleration in m/s/s predicted using vehicle and wind velocity estimates
        // Initialised to measured value and updated later using available drag model
        ftype predAccel = mea_acc;

        // predicted sign of drag force
        const ftype dragForceSign = is_positive(rel_wind_body[axis_index]) ? -1.0f : 1.0f;

        if (axis_index == 0) {
            // drag can be modelled as an arbitrary  combination of bluff body drag that proportional to
            // speed squared, and rotor momentum drag that is proportional to speed.
            ftype Kacc; // Derivative of specific force wrt airspeed
            if (using_mcoef && using_bcoef_x) {
                // mixed bluff body and propeller momentum drag
                const ftype airSpd = (bcoef_x / rho) * (- mcoef + sqrtF(sq(mcoef) + 2.0f * (rho / bcoef_x) * fabsF(mea_acc)));
                Kacc = fmaxF(1e-1f, (rho / bcoef_x) * airSpd + mcoef * density_ratio);
                predAccel = (0.5f / bcoef_x) * rho * sq(rel_wind_body[0]) * dragForceSign - rel_wind_body[0] * mcoef * density_ratio;
            } else if (using_mcoef) {
                // propeller momentum drag only
                Kacc = fmaxF(1e-1f, mcoef * density_ratio);
                predAccel = - rel_wind_body[0] * mcoef * density_ratio;
            } else if (using_bcoef_x) {
                // bluff body drag only
                const ftype airSpd = sqrtF((2.0f * bcoef_x * fabsF(mea_acc)) / rho);
                Kacc = fmaxF(1e-1f, (rho / bcoef_x) * airSpd);
                predAccel = (0.5f / bcoef_x) * rho * sq(rel_wind_body[0]) * dragForceSign;
            } else {
                // skip this axis
                continue;
            }

            // intermediate variables
            const ftype HK0 = vn - vwn;
            const ftype HK1 = ve - vwe;
            const ftype HK2 = HK0*q0 + HK1*q3 - q2*vd;
            const ftype HK3 = 2*Kacc;
            const ftype HK4 = HK0*q1 + HK1*q2 + q3*vd;
            const ftype HK5 = HK0*q2 - HK1*q1 + q0*vd;
            const ftype HK6 = -HK0*q3 + HK1*q0 + q1*vd;
            const ftype HK7 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
            const ftype HK8 = HK7*Kacc;
            const ftype HK9 = q0*q3 + q1*q2;
            const ftype HK10 = HK3*HK9;
            const ftype HK11 = q0*q2 - q1*q3;
            const ftype HK12 = 2*HK9;
            const ftype HK13 = 2*HK11;
            const ftype HK14 = 2*HK4;
            const ftype HK15 = 2*HK2;
            const ftype HK16 = 2*HK5;
            const ftype HK17 = 2*HK6;
            const ftype HK18 = -HK12*P[0][23] + HK12*P[0][5] - HK13*P[0][6] + HK14*P[0][1] + HK15*P[0][0] - HK16*P[0][2] + HK17*P[0][3] - HK7*P[0][22] + HK7*P[0][4];
            const ftype HK19 = HK12*P[5][23];
            const ftype HK20 = -HK12*P[23][23] - HK13*P[6][23] + HK14*P[1][23] + HK15*P[0][23] - HK16*P[2][23] + HK17*P[3][23] + HK19 - HK7*P[22][23] + HK7*P[4][23];
            const ftype HK21 = sq(Kacc);
            const ftype HK22 = HK12*HK21;
            const ftype HK23 = HK12*P[5][5] - HK13*P[5][6] + HK14*P[1][5] + HK15*P[0][5] - HK16*P[2][5] + HK17*P[3][5] - HK19 + HK7*P[4][5] - HK7*P[5][22];
            const ftype HK24 = HK12*P[5][6] - HK12*P[6][23] - HK13*P[6][6] + HK14*P[1][6] + HK15*P[0][6] - HK16*P[2][6] + HK17*P[3][6] + HK7*P[4][6] - HK7*P[6][22];
            const ftype HK25 = HK7*P[4][22];
            const ftype HK26 = -HK12*P[4][23] + HK12*P[4][5] - HK13*P[4][6] + HK14*P[1][4] + HK15*P[0][4] - HK16*P[2][4] + HK17*P[3][4] - HK25 + HK7*P[4][4];
            const ftype HK27 = HK21*HK7;
            const ftype HK28 = -HK12*P[22][23] + HK12*P[5][22] - HK13*P[6][22] + HK14*P[1][22] + HK15*P[0][22] - HK16*P[2][22] + HK17*P[3][22] + HK25 - HK7*P[22][22];
            const ftype HK29 = -HK12*P[1][23] + HK12*P[1][5] - HK13*P[1][6] + HK14*P[1][1] + HK15*P[0][1] - HK16*P[1][2] + HK17*P[1][3] - HK7*P[1][22] + HK7*P[1][4];
            const ftype HK30 = -HK12*P[2][23] + HK12*P[2][5] - HK13*P[2][6] + HK14*P[1][2] + HK15*P[0][2] - HK16*P[2][2] + HK17*P[2][3] - HK7*P[2][22] + HK7*P[2][4];
            const ftype HK31 = -HK12*P[3][23] + HK12*P[3][5] - HK13*P[3][6] + HK14*P[1][3] + HK15*P[0][3] - HK16*P[2][3] + HK17*P[3][3] - HK7*P[3][22] + HK7*P[3][4];
            // const ftype HK32 = Kacc/(-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);

            // calculate innovation variance and exit if badly conditioned
            innovDragVar.x = (-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);
            if (innovDragVar.x < R_ACC) {
                return;
            }
            const ftype HK32 = Kacc / innovDragVar.x;

            // Observation Jacobians
            Hfusion[0] = -HK2*HK3;
            Hfusion[1] = -HK3*HK4;
            Hfusion[2] = HK3*HK5;
            Hfusion[3] = -HK3*HK6;
            Hfusion[4] = -HK8;
            Hfusion[5] = -HK10;
            Hfusion[6] = HK11*HK3;
            Hfusion[22] = HK8;
            Hfusion[23] = HK10;

            // Kalman gains
            // Don't allow modification of any states other than wind velocity - we only need a wind estimate.
            // See AP_NavEKF3/derivation/generated/acc_bf_generated.cpp for un-implemented Kalman gain equations.
            Kfusion[22] = -HK28*HK32;
            Kfusion[23] = -HK20*HK32;


        } else if (axis_index == 1) {
            // drag can be modelled as an arbitrary combination of bluff body drag that proportional to
            // speed squared, and rotor momentum drag that is proportional to speed.
            ftype Kacc; // Derivative of specific force wrt airspeed
            if (using_mcoef && using_bcoef_y) {
                // mixed bluff body and propeller momentum drag
                const ftype airSpd = (bcoef_y / rho) * (- mcoef + sqrtF(sq(mcoef) + 2.0f * (rho / bcoef_y) * fabsF(mea_acc)));
                Kacc = fmaxF(1e-1f, (rho / bcoef_y) * airSpd + mcoef * density_ratio);
                predAccel = (0.5f / bcoef_y) * rho * sq(rel_wind_body[1]) * dragForceSign - rel_wind_body[1] * mcoef * density_ratio;
            } else if (using_mcoef) {
                // propeller momentum drag only
                Kacc = fmaxF(1e-1f, mcoef * density_ratio);
                predAccel = - rel_wind_body[1] * mcoef * density_ratio;
            } else if (using_bcoef_y) {
                // bluff body drag only
                const ftype airSpd = sqrtF((2.0f * bcoef_y * fabsF(mea_acc)) / rho);
                Kacc = fmaxF(1e-1f, (rho / bcoef_y) * airSpd);
                predAccel = (0.5f / bcoef_y) * rho * sq(rel_wind_body[1]) * dragForceSign;
            } else {
                // nothing more to do
                return;
            }

            // intermediate variables
            const ftype HK0 = ve - vwe;
            const ftype HK1 = vn - vwn;
            const ftype HK2 = HK0*q0 - HK1*q3 + q1*vd;
            const ftype HK3 = 2*Kacc;
            const ftype HK4 = -HK0*q1 + HK1*q2 + q0*vd;
            const ftype HK5 = HK0*q2 + HK1*q1 + q3*vd;
            const ftype HK6 = HK0*q3 + HK1*q0 - q2*vd;
            const ftype HK7 = q0*q3 - q1*q2;
            const ftype HK8 = HK3*HK7;
            const ftype HK9 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
            const ftype HK10 = HK9*Kacc;
            const ftype HK11 = q0*q1 + q2*q3;
            const ftype HK12 = 2*HK11;
            const ftype HK13 = 2*HK7;
            const ftype HK14 = 2*HK5;
            const ftype HK15 = 2*HK2;
            const ftype HK16 = 2*HK4;
            const ftype HK17 = 2*HK6;
            const ftype HK18 = HK12*P[0][6] + HK13*P[0][22] - HK13*P[0][4] + HK14*P[0][2] + HK15*P[0][0] + HK16*P[0][1] - HK17*P[0][3] - HK9*P[0][23] + HK9*P[0][5];
            const ftype HK19 = sq(Kacc);
            const ftype HK20 = HK12*P[6][6] - HK13*P[4][6] + HK13*P[6][22] + HK14*P[2][6] + HK15*P[0][6] + HK16*P[1][6] - HK17*P[3][6] + HK9*P[5][6] - HK9*P[6][23];
            const ftype HK21 = HK13*P[4][22];
            const ftype HK22 = HK12*P[6][22] + HK13*P[22][22] + HK14*P[2][22] + HK15*P[0][22] + HK16*P[1][22] - HK17*P[3][22] - HK21 - HK9*P[22][23] + HK9*P[5][22];
            const ftype HK23 = HK13*HK19;
            const ftype HK24 = HK12*P[4][6] - HK13*P[4][4] + HK14*P[2][4] + HK15*P[0][4] + HK16*P[1][4] - HK17*P[3][4] + HK21 - HK9*P[4][23] + HK9*P[4][5];
            const ftype HK25 = HK9*P[5][23];
            const ftype HK26 = HK12*P[5][6] - HK13*P[4][5] + HK13*P[5][22] + HK14*P[2][5] + HK15*P[0][5] + HK16*P[1][5] - HK17*P[3][5] - HK25 + HK9*P[5][5];
            const ftype HK27 = HK19*HK9;
            const ftype HK28 = HK12*P[6][23] + HK13*P[22][23] - HK13*P[4][23] + HK14*P[2][23] + HK15*P[0][23] + HK16*P[1][23] - HK17*P[3][23] + HK25 - HK9*P[23][23];
            const ftype HK29 = HK12*P[2][6] + HK13*P[2][22] - HK13*P[2][4] + HK14*P[2][2] + HK15*P[0][2] + HK16*P[1][2] - HK17*P[2][3] - HK9*P[2][23] + HK9*P[2][5];
            const ftype HK30 = HK12*P[1][6] + HK13*P[1][22] - HK13*P[1][4] + HK14*P[1][2] + HK15*P[0][1] + HK16*P[1][1] - HK17*P[1][3] - HK9*P[1][23] + HK9*P[1][5];
            const ftype HK31 = HK12*P[3][6] + HK13*P[3][22] - HK13*P[3][4] + HK14*P[2][3] + HK15*P[0][3] + HK16*P[1][3] - HK17*P[3][3] - HK9*P[3][23] + HK9*P[3][5];
            // const ftype HK32 = Kaccy/(HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);

            innovDragVar.y = (HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);
            if (innovDragVar.y < R_ACC) {
                // calculation is badly conditioned
                return;
            }
            const ftype HK32 = Kacc / innovDragVar.y;

            // Observation Jacobians
            Hfusion[0] = -HK2*HK3;
            Hfusion[1] = -HK3*HK4;
            Hfusion[2] = -HK3*HK5;
            Hfusion[3] = HK3*HK6;
            Hfusion[4] = HK8;
            Hfusion[5] = -HK10;
            Hfusion[6] = -HK11*HK3;
            Hfusion[22] = -HK8;
            Hfusion[23] = HK10;

            // Kalman gains
            // Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
            // See AP_NavEKF3/derivation/generated/acc_bf_generated.cpp for un-implemented Kalman gain equations.
            Kfusion[22] = -HK22*HK32;
            Kfusion[23] = -HK28*HK32;
        }

        innovDrag[axis_index] = predAccel - mea_acc;
        dragTestRatio[axis_index] = sq(innovDrag[axis_index]) / (25.0f * innovDragVar[axis_index]);

        // if the innovation consistency check fails then don't fuse the sample
        if (dragTestRatio[axis_index] > 1.0f) {
            return;
        }

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovDrag[axis_index];
        }
        stateStruct.quat.normalize();

        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=6; j++) {
                KH[i][j] = Kfusion[i] * Hfusion[j];
            }
            for (unsigned j = 7; j<=21; j++) {
                KH[i][j] = 0.0f;
            }
            for (unsigned j = 22; j<=23; j++) {
                KH[i][j] = Kfusion[i] * Hfusion[j];
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

    // record time of successful fusion
    lastDragPassTime_ms = imuSampleTime_ms;
}
#endif // EK3_FEATURE_DRAG_FUSION

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

