#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of optical flow measurements
void NavEKF3_core::SelectFlowFusion()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseOptFlow);

    // Check for data at the fusion time horizon
    flowDataToFuse = storedOF.recall(ofDataDelayed, imuDataDelayed.time_ms);

    // Check if the flow sensor is orientated such that it can use the terrain height as a reference
    if (flowDataToFuse) {
        // Get the rotation matrix from body frame to sensor frame
        Tbs_flow = *ofDataDelayed.Tbs;
        // If the flow sensor is not aligned with the Z body axis then it cannot use the terrain height as a reference
        flowUseTerrainNo = (Tbs_flow.c.z < 0.95f);
    }

    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !optFlowFusionDelayed) {
        optFlowFusionDelayed = true;
        return;
    } else {
        optFlowFusionDelayed = false;
    }

    // Perform Data Checks
    // Check if the optical flow data is still valid
    flowDataValid = ((imuSampleTime_ms - flowValidMeaTime_ms) < 1000);

    // check is the terrain offset estimate is still valid - if we are using range finder as the main height reference, the ground is assumed to be at 0
    gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000) || (activeHgtSource == HGT_SOURCE_RNG);

    // Perform tilt check
    bool tiltOK = (prevTnb.c.z > frontend->DCM33FlowMin);

    // Constrain measurements to zero if takeoff is not detected and the height above ground
    // is insuffient to achieve acceptable focus. This allows the vehicle to be picked up
    // and carried to test optical flow operation
    if (!takeOffDetected && ((terrainState - stateStruct.position.z) < 0.5f)) {
        ofDataDelayed.flowRadXYcomp.zero();
        ofDataDelayed.flowRadXY.zero();
        flowDataValid = true;
    }

    // if we do have valid flow measurements, fuse data into a 1-state EKF to estimate terrain height
    // we don't do terrain height estimation in optical flow only mode as the ground becomes our zero height reference
    if ((flowDataToFuse || rangeDataToFuse) && tiltOK) {
        // fuse optical flow data into the terrain estimator if available, the flow sensor can use the terrain height as a reference and there is no range data (range data is better)
         fuseOptFlowData = (flowDataToFuse && !flowUseTerrainNo && !rangeDataToFuse);
        // Estimate the terrain offset (runs a one state EKF)
        EstimateTerrainOffset();
    }

    // Fuse optical flow data into the main filter
    if (flowDataToFuse && (tiltOK || flowUseTerrainNo)) {
        // Set the flow noise used by the fusion processes
        R_LOS = sq(MAX(frontend->_flowNoise, 0.05f));
        // Fuse the optical flow X and Y axis data into the main filter sequentially
        FuseOptFlow();
        // reset flag to indicate that no new flow data is available for fusion
        flowDataToFuse = false;
    }

    // stop the performance timer
    hal.util->perf_end(_perf_FuseOptFlow);
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optiocal flow rates and range finder measurements
*/
void NavEKF3_core::EstimateTerrainOffset()
{
    // start performance timer
    hal.util->perf_begin(_perf_TerrainOffset);

    // constrain height above ground to be above range measured on ground
    float heightAboveGndEst = MAX((terrainState - stateStruct.position.z), rngOnGnd);

    // calculate a predicted LOS rate squared
    float velHorizSq = sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y);
    float losRateSq = velHorizSq / sq(heightAboveGndEst);

    // don't update terrain offset state if there is no range finder
    // don't update terrain state if not generating enough LOS rate, or without GPS, as it is poorly observable
    // don't update terrain state if we are using it as a height reference in the main filter
    bool cantFuseFlowData = (gpsNotAvailable || PV_AidingMode == AID_RELATIVE || velHorizSq < 25.0f || losRateSq < 0.01f);
    if ((!rangeDataToFuse && cantFuseFlowData) || (activeHgtSource == HGT_SOURCE_RNG)) {
        // skip update
        inhibitGndState = true;
    } else {
        inhibitGndState = false;
        // record the time we last updated the terrain offset state
        gndHgtValidTime_ms = imuSampleTime_ms;

        // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
        // limit distance to prevent intialisation afer bad gps causing bad numerical conditioning
        float distanceTravelledSq = sq(stateStruct.position[0] - prevPosN) + sq(stateStruct.position[1] - prevPosE);
        distanceTravelledSq = MIN(distanceTravelledSq, 100.0f);
        prevPosN = stateStruct.position[0];
        prevPosE = stateStruct.position[1];

        // in addition to a terrain gradient error model, we also have the growth in uncertainty due to the copters vertical velocity
        float timeLapsed = MIN(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f);
        float Pincrement = (distanceTravelledSq * sq(0.01f*float(frontend->gndGradientSigma))) + sq(timeLapsed)*P[6][6];
        Popt += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (rangeDataToFuse) {
            // predict range
            float predRngMeas = MAX((terrainState - stateStruct.position[2]),rngOnGnd) / prevTnb.c.z;

            // Copy required states to local variable names
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            float R_RNG = frontend->_rngNoise;

            // calculate Kalman gain
            float SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            float K_RNG = Popt/(SK_RNG*(R_RNG + Popt/sq(SK_RNG)));

            // Calculate the innovation variance for data logging
            varInnovRng = (R_RNG + Popt/sq(SK_RNG));

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

            // Calculate the measurement innovation
            innovRng = predRngMeas - rangeDataDelayed.rng;

            // calculate the innovation consistency test ratio
            auxRngTestRatio = sq(innovRng) / (sq(MAX(0.01f * (float)frontend->_rngInnovGate, 1.0f)) * varInnovRng);

            // Check the innovation test ratio and don't fuse if too large
            if (auxRngTestRatio < 1.0f) {
                // correct the state
                terrainState -= K_RNG * innovRng;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - sq(Popt)/(SK_RNG*(R_RNG + Popt/sq(SK_RNG))*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));

                // prevent the state variance from becoming negative
                Popt = MAX(Popt,0.0f);

            }
        }

        if (fuseOptFlowData && !cantFuseFlowData) {

            Vector3f relVelSensor; // velocity of sensor relative to ground in sensor axes
            float losPred; // predicted optical flow angular rate measurement
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time
            float K_OPT;
            float H_OPT;

            // predict range to centre of image
            float flowRngPred = MAX((terrainState - stateStruct.position[2]),rngOnGnd) / prevTnb.c.z;

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

            // calculate relative velocity in sensor frame
            relVelSensor = prevTnb*stateStruct.velocity;

            // divide velocity by range, subtract body rates and apply scale factor to
            // get predicted sensed angular optical rates relative to X and Y sensor axes
            losPred =   norm(relVelSensor.x, relVelSensor.y)/flowRngPred;

            // calculate innovations
            auxFlowObsInnov = losPred - norm(ofDataDelayed.flowRadXYcomp.x, ofDataDelayed.flowRadXYcomp.y);

            // calculate observation jacobian
            float t3 = sq(q0);
            float t4 = sq(q1);
            float t5 = sq(q2);
            float t6 = sq(q3);
            float t10 = q0*q3*2.0f;
            float t11 = q1*q2*2.0f;
            float t14 = t3+t4-t5-t6;
            float t15 = t14*stateStruct.velocity.x;
            float t16 = t10+t11;
            float t17 = t16*stateStruct.velocity.y;
            float t18 = q0*q2*2.0f;
            float t19 = q1*q3*2.0f;
            float t20 = t18-t19;
            float t21 = t20*stateStruct.velocity.z;
            float t2 = t15+t17-t21;
            float t7 = t3-t4-t5+t6;
            float t8 = stateStruct.position[2]-terrainState;
            float t9 = 1.0f/sq(t8);
            float t24 = t3-t4+t5-t6;
            float t25 = t24*stateStruct.velocity.y;
            float t26 = t10-t11;
            float t27 = t26*stateStruct.velocity.x;
            float t28 = q0*q1*2.0f;
            float t29 = q2*q3*2.0f;
            float t30 = t28+t29;
            float t31 = t30*stateStruct.velocity.z;
            float t12 = t25-t27+t31;
            float t13 = sq(t7);
            float t22 = sq(t2);
            float t23 = 1.0f/(t8*t8*t8);
            float t32 = sq(t12);
            H_OPT = 0.5f*(t13*t22*t23*2.0f+t13*t23*t32*2.0f)/sqrtf(t9*t13*t22+t9*t13*t32);

            // calculate innovation variances
            auxFlowObsInnovVar = H_OPT*Popt*H_OPT + R_LOS;

            // calculate Kalman gain
            K_OPT = Popt*H_OPT/auxFlowObsInnovVar;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio = sq(auxFlowObsInnov) / (sq(MAX(0.01f * (float)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar);

            // don't fuse if optical flow data is outside valid range
            if (MAX(ofDataDelayed.flowRadXY[0],ofDataDelayed.flowRadXY[1]) < frontend->_maxFlowRate) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming negative
                Popt = MAX(Popt,0.0f);
            }
        }
    }

    // stop the performance timer
    hal.util->perf_end(_perf_TerrainOffset);
}

/*
 * Fuse angular motion compensated optical flow rates using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here at commit 1615442:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * Requires a valid terrain height estimate.
*/
void NavEKF3_core::FuseOptFlow()
{
    Vector24 H_LOS;
    Vector3f relVelSensor;
    Vector2 losPred;

    // Copy required states to local variable names
    float q0  = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];
    float vn = stateStruct.velocity.x;
    float ve = stateStruct.velocity.y;
    float vd = stateStruct.velocity.z;
    float pd = stateStruct.position.z;

    // if using downwards pointing flow sensor calculate range from ground plain to centre of sensor fov assuming flat earth
    // otherwise use range measurement directly
    float range;
    Vector3f posOffsetBody = (*ofDataDelayed.body_offset) - accelPosOffset;
    if (!flowUseTerrainNo) {
        // constrain height above ground to be above range measured on ground
        float heightAboveGndEst = MAX((terrainState - pd), rngOnGnd);
        range = constrain_float((heightAboveGndEst/prevTnb.c.z),rngOnGnd,1000.0f);
        // correct range for flow sensor offset body frame position offset
        // the corrected value is the predicted range from the sensor focal point to the
        // centre of the image on the ground assuming flat terrain
        if (!posOffsetBody.is_zero()) {
            Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
            range -= posOffsetEarth.z / prevTnb.c.z;
        }
    } else if (ofDataDelayed.range > 0.01f) {
        range = ofDataDelayed.range;
    } else {
        return;
    }

    // calculate relative velocity in body frame including the relative motion due to rotation
    // and rotate into sensor frame
    relVelSensor = Tbs_flow * ((prevTnb * stateStruct.velocity) + (ofDataDelayed.bodyRadXYZ % posOffsetBody));

    // divide velocity by range  to get predicted angular LOS rates relative to X and Y axes
    losPred[0] =  relVelSensor.y/range;
    losPred[1] = -relVelSensor.x/range;

    // Fuse X and Y axis measurements sequentially assuming observation errors are uncorrelated
    for (uint8_t obsIndex=0; obsIndex<=1; obsIndex++) { // fuse X axis data first

        // calculate observation jacobians and Kalman gains
        memset(&H_LOS[0], 0, sizeof(H_LOS));
        if (obsIndex == 0) {
            // calculate X axis observation Jacobian and Kalman gain vectors
            // code fragments from https://github.com/PX4/ecl/blob/master/matlab/scripts/LOSX.c
            // at commit 1615442
            float t2 = 1.0f/range;
            float t3 = Tbs_flow.b.y*q0*2.0f;
            float t4 = Tbs_flow.b.x*q3*2.0f;
            float t18 = Tbs_flow.b.z*q1*2.0f;
            float t5 = t3+t4-t18;
            float t6 = Tbs_flow.b.y*q1*2.0f;
            float t7 = Tbs_flow.b.z*q0*2.0f;
            float t16 = Tbs_flow.b.x*q2*2.0f;
            float t8 = t6+t7-t16;
            float t9 = Tbs_flow.b.x*q0*2.0f;
            float t10 = Tbs_flow.b.z*q2*2.0f;
            float t17 = Tbs_flow.b.y*q3*2.0f;
            float t11 = t9+t10-t17;
            float t12 = Tbs_flow.b.x*q1*2.0f;
            float t13 = Tbs_flow.b.y*q2*2.0f;
            float t14 = Tbs_flow.b.z*q3*2.0f;
            float t15 = t12+t13+t14;
            float t19 = q0*q0;
            float t20 = q1*q1;
            float t21 = q2*q2;
            float t22 = q3*q3;
            float t23 = q0*q3*2.0f;
            float t24 = q0*q2*2.0f;
            float t25 = q1*q3*2.0f;
            float t26 = q0*q1*2.0f;
            float t27 = t19+t20-t21-t22;
            float t28 = Tbs_flow.b.x*t27;
            float t29 = q1*q2*2.0f;
            float t30 = t24+t25;
            float t31 = Tbs_flow.b.z*t30;
            float t32 = t19-t20+t21-t22;
            float t33 = Tbs_flow.b.y*t32;
            float t34 = t23+t29;
            float t35 = Tbs_flow.b.x*t34;
            float t36 = q2*q3*2.0f;
            float t37 = t19-t20-t21+t22;
            float t38 = Tbs_flow.b.z*t37;
            float t39 = t24-t25;
            float t40 = t26+t36;
            float t41 = Tbs_flow.b.y*t40;
            float t60 = Tbs_flow.b.x*t39;
            float t42 = t38+t41-t60;
            float t43 = t8*vd;
            float t44 = t5*ve;
            float t45 = t11*vn;
            float t46 = t43+t44+t45;
            float t47 = t5*vd;
            float t48 = t15*vn;
            float t62 = t8*ve;
            float t49 = t47+t48-t62;
            float t50 = t15*ve;
            float t51 = t8*vn;
            float t63 = t11*vd;
            float t52 = t50+t51-t63;
            float t53 = t15*vd;
            float t54 = t11*ve;
            float t64 = t5*vn;
            float t55 = t53+t54-t64;
            float t56 = t23-t29;
            float t65 = Tbs_flow.b.y*t56;
            float t57 = t28+t31-t65;
            float t58 = t26-t36;
            float t66 = Tbs_flow.b.z*t58;
            float t59 = t33+t35-t66;
            float t61 = P[0][0]*t2*t46;
            float t67 = P[1][1]*t2*t49;
            float t68 = P[4][0]*t2*t57;
            float t69 = P[5][0]*t2*t59;
            float t70 = P[6][0]*t2*t42;
            float t71 = P[1][0]*t2*t49;
            float t72 = P[2][0]*t2*t52;
            float t73 = P[3][0]*t2*t55;
            float t74 = t61+t68+t69+t70+t71+t72+t73;
            float t75 = t2*t46*t74;
            float t76 = P[4][1]*t2*t57;
            float t77 = P[5][1]*t2*t59;
            float t78 = P[6][1]*t2*t42;
            float t79 = P[0][1]*t2*t46;
            float t80 = P[2][1]*t2*t52;
            float t81 = P[3][1]*t2*t55;
            float t82 = t67+t76+t77+t78+t79+t80+t81;
            float t83 = t2*t49*t82;
            float t84 = P[4][2]*t2*t57;
            float t85 = P[5][2]*t2*t59;
            float t86 = P[6][2]*t2*t42;
            float t87 = P[0][2]*t2*t46;
            float t88 = P[1][2]*t2*t49;
            float t89 = P[2][2]*t2*t52;
            float t90 = P[3][2]*t2*t55;
            float t91 = t84+t85+t86+t87+t88+t89+t90;
            float t92 = t2*t52*t91;
            float t93 = P[4][3]*t2*t57;
            float t94 = P[5][3]*t2*t59;
            float t95 = P[6][3]*t2*t42;
            float t96 = P[0][3]*t2*t46;
            float t97 = P[1][3]*t2*t49;
            float t98 = P[2][3]*t2*t52;
            float t99 = P[3][3]*t2*t55;
            float t100 = t93+t94+t95+t96+t97+t98+t99;
            float t101 = t2*t55*t100;
            float t102 = P[4][4]*t2*t57;
            float t103 = P[5][4]*t2*t59;
            float t104 = P[6][4]*t2*t42;
            float t105 = P[0][4]*t2*t46;
            float t106 = P[1][4]*t2*t49;
            float t107 = P[2][4]*t2*t52;
            float t108 = P[3][4]*t2*t55;
            float t109 = t102+t103+t104+t105+t106+t107+t108;
            float t110 = t2*t57*t109;
            float t111 = P[4][5]*t2*t57;
            float t112 = P[5][5]*t2*t59;
            float t113 = P[6][5]*t2*t42;
            float t114 = P[0][5]*t2*t46;
            float t115 = P[1][5]*t2*t49;
            float t116 = P[2][5]*t2*t52;
            float t117 = P[3][5]*t2*t55;
            float t118 = t111+t112+t113+t114+t115+t116+t117;
            float t119 = t2*t59*t118;
            float t120 = P[4][6]*t2*t57;
            float t121 = P[5][6]*t2*t59;
            float t122 = P[6][6]*t2*t42;
            float t123 = P[0][6]*t2*t46;
            float t124 = P[1][6]*t2*t49;
            float t125 = P[2][6]*t2*t52;
            float t126 = P[3][6]*t2*t55;
            float t127 = t120+t121+t122+t123+t124+t125+t126;
            float t128 = t2*t42*t127;
            float t129 = R_LOS+t75+t83+t92+t101+t110+t119+t128;
            float t130;

            H_LOS[0] = t2*t46;
            H_LOS[1] = t2*t49;
            H_LOS[2] = t2*t52;
            H_LOS[3] = t2*t55;
            H_LOS[4] = t2*(t28+t31-Tbs_flow.b.y*(t23-q1*q2*2.0f));
            H_LOS[5] = t2*(t33+t35-Tbs_flow.b.z*(t26-q2*q3*2.0f));
            H_LOS[6] = t2*t42;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t129 > R_LOS) {
                t130 = 1.0f/t129;
                faultStatus.bad_xflow = false;
            } else {
                t129 = R_LOS;
                t130 = 1.0f/R_LOS;
                faultStatus.bad_xflow = true;
                return;
            }
            varInnovOptFlow[0] = t129;

            // calculate innovation for X axis observation
            innovOptFlow[0] = losPred[0] - ofDataDelayed.flowRadXYcomp.x;

            // calculate Kalman gains for X-axis observation
            Kfusion[0] = t130*(t61+P[0][6]*t2*t42+P[0][1]*t2*t49+P[0][2]*t2*t52+P[0][3]*t2*t55+P[0][4]*t2*t57+P[0][5]*t2*t59);
            Kfusion[1] = t130*(t67+P[1][0]*t2*t46+P[1][6]*t2*t42+P[1][2]*t2*t52+P[1][3]*t2*t55+P[1][4]*t2*t57+P[1][5]*t2*t59);
            Kfusion[2] = t130*(t89+P[2][0]*t2*t46+P[2][6]*t2*t42+P[2][1]*t2*t49+P[2][3]*t2*t55+P[2][4]*t2*t57+P[2][5]*t2*t59);
            Kfusion[3] = t130*(t99+P[3][0]*t2*t46+P[3][6]*t2*t42+P[3][1]*t2*t49+P[3][2]*t2*t52+P[3][4]*t2*t57+P[3][5]*t2*t59);
            Kfusion[4] = t130*(t102+P[4][0]*t2*t46+P[4][6]*t2*t42+P[4][1]*t2*t49+P[4][2]*t2*t52+P[4][3]*t2*t55+P[4][5]*t2*t59);
            Kfusion[5] = t130*(t112+P[5][0]*t2*t46+P[5][6]*t2*t42+P[5][1]*t2*t49+P[5][2]*t2*t52+P[5][3]*t2*t55+P[5][4]*t2*t57);
            Kfusion[6] = t130*(t122+P[6][0]*t2*t46+P[6][1]*t2*t49+P[6][2]*t2*t52+P[6][3]*t2*t55+P[6][4]*t2*t57+P[6][5]*t2*t59);
            Kfusion[7] = t130*(P[7][0]*t2*t46+P[7][6]*t2*t42+P[7][1]*t2*t49+P[7][2]*t2*t52+P[7][3]*t2*t55+P[7][4]*t2*t57+P[7][5]*t2*t59);
            Kfusion[8] = t130*(P[8][0]*t2*t46+P[8][6]*t2*t42+P[8][1]*t2*t49+P[8][2]*t2*t52+P[8][3]*t2*t55+P[8][4]*t2*t57+P[8][5]*t2*t59);
            Kfusion[9] = t130*(P[9][0]*t2*t46+P[9][6]*t2*t42+P[9][1]*t2*t49+P[9][2]*t2*t52+P[9][3]*t2*t55+P[9][4]*t2*t57+P[9][5]*t2*t59);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = t130*(P[10][0]*t2*t46+P[10][6]*t2*t42+P[10][1]*t2*t49+P[10][2]*t2*t52+P[10][3]*t2*t55+P[10][4]*t2*t57+P[10][5]*t2*t59);
                Kfusion[11] = t130*(P[11][0]*t2*t46+P[11][6]*t2*t42+P[11][1]*t2*t49+P[11][2]*t2*t52+P[11][3]*t2*t55+P[11][4]*t2*t57+P[11][5]*t2*t59);
                Kfusion[12] = t130*(P[12][0]*t2*t46+P[12][6]*t2*t42+P[12][1]*t2*t49+P[12][2]*t2*t52+P[12][3]*t2*t55+P[12][4]*t2*t57+P[12][5]*t2*t59);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = t130*(P[13][0]*t2*t46+P[13][6]*t2*t42+P[13][1]*t2*t49+P[13][2]*t2*t52+P[13][3]*t2*t55+P[13][4]*t2*t57+P[13][5]*t2*t59);
                Kfusion[14] = t130*(P[14][0]*t2*t46+P[14][6]*t2*t42+P[14][1]*t2*t49+P[14][2]*t2*t52+P[14][3]*t2*t55+P[14][4]*t2*t57+P[14][5]*t2*t59);
                Kfusion[15] = t130*(P[15][0]*t2*t46+P[15][6]*t2*t42+P[15][1]*t2*t49+P[15][2]*t2*t52+P[15][3]*t2*t55+P[15][4]*t2*t57+P[15][5]*t2*t59);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = t130*(P[16][0]*t2*t46+P[16][6]*t2*t42+P[16][1]*t2*t49+P[16][2]*t2*t52+P[16][3]*t2*t55+P[16][4]*t2*t57+P[16][5]*t2*t59);
                Kfusion[17] = t130*(P[17][0]*t2*t46+P[17][6]*t2*t42+P[17][1]*t2*t49+P[17][2]*t2*t52+P[17][3]*t2*t55+P[17][4]*t2*t57+P[17][5]*t2*t59);
                Kfusion[18] = t130*(P[18][0]*t2*t46+P[18][6]*t2*t42+P[18][1]*t2*t49+P[18][2]*t2*t52+P[18][3]*t2*t55+P[18][4]*t2*t57+P[18][5]*t2*t59);
                Kfusion[19] = t130*(P[19][0]*t2*t46+P[19][6]*t2*t42+P[19][1]*t2*t49+P[19][2]*t2*t52+P[19][3]*t2*t55+P[19][4]*t2*t57+P[19][5]*t2*t59);
                Kfusion[20] = t130*(P[20][0]*t2*t46+P[20][6]*t2*t42+P[20][1]*t2*t49+P[20][2]*t2*t52+P[20][3]*t2*t55+P[20][4]*t2*t57+P[20][5]*t2*t59);
                Kfusion[21] = t130*(P[21][0]*t2*t46+P[21][6]*t2*t42+P[21][1]*t2*t49+P[21][2]*t2*t52+P[21][3]*t2*t55+P[21][4]*t2*t57+P[21][5]*t2*t59);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = t130*(P[22][0]*t2*t46+P[22][6]*t2*t42+P[22][1]*t2*t49+P[22][2]*t2*t52+P[22][3]*t2*t55+P[22][4]*t2*t57+P[22][5]*t2*t59);
                Kfusion[23] = t130*(P[23][0]*t2*t46+P[23][6]*t2*t42+P[23][1]*t2*t49+P[23][2]*t2*t52+P[23][3]*t2*t55+P[23][4]*t2*t57+P[23][5]*t2*t59);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }

        } else {
            // calculate Y axis observation Jacobian and Kalman gain vectors
            // code fragments from https://github.com/PX4/ecl/blob/master/matlab/scripts/LOSY.c
            // at commit 1615442
            float t2 = 1.0f/range;
            float t3 = Tbs_flow.a.y*q0*2.0f;
            float t4 = Tbs_flow.a.x*q3*2.0f;
            float t18 = Tbs_flow.a.z*q1*2.0f;
            float t5 = t3+t4-t18;
            float t6 = Tbs_flow.a.y*q1*2.0f;
            float t7 = Tbs_flow.a.z*q0*2.0f;
            float t16 = Tbs_flow.a.x*q2*2.0f;
            float t8 = t6+t7-t16;
            float t9 = Tbs_flow.a.x*q0*2.0f;
            float t10 = Tbs_flow.a.z*q2*2.0f;
            float t17 = Tbs_flow.a.y*q3*2.0f;
            float t11 = t9+t10-t17;
            float t12 = Tbs_flow.a.x*q1*2.0f;
            float t13 = Tbs_flow.a.y*q2*2.0f;
            float t14 = Tbs_flow.a.z*q3*2.0f;
            float t15 = t12+t13+t14;
            float t19 = q0*q0;
            float t20 = q1*q1;
            float t21 = q2*q2;
            float t22 = q3*q3;
            float t23 = q0*q3*2.0f;
            float t24 = q0*q2*2.0f;
            float t25 = q1*q3*2.0f;
            float t26 = q0*q1*2.0f;
            float t27 = t19+t20-t21-t22;
            float t28 = Tbs_flow.a.x*t27;
            float t29 = q1*q2*2.0f;
            float t30 = t24+t25;
            float t31 = Tbs_flow.a.z*t30;
            float t32 = t19-t20+t21-t22;
            float t33 = Tbs_flow.a.y*t32;
            float t34 = t23+t29;
            float t35 = Tbs_flow.a.x*t34;
            float t36 = q2*q3*2.0f;
            float t37 = t19-t20-t21+t22;
            float t38 = Tbs_flow.a.z*t37;
            float t39 = t24-t25;
            float t40 = t26+t36;
            float t41 = Tbs_flow.a.y*t40;
            float t60 = Tbs_flow.a.x*t39;
            float t42 = t38+t41-t60;
            float t43 = t8*vd;
            float t44 = t5*ve;
            float t45 = t11*vn;
            float t46 = t43+t44+t45;
            float t47 = t5*vd;
            float t48 = t15*vn;
            float t62 = t8*ve;
            float t49 = t47+t48-t62;
            float t50 = t15*ve;
            float t51 = t8*vn;
            float t63 = t11*vd;
            float t52 = t50+t51-t63;
            float t53 = t15*vd;
            float t54 = t11*ve;
            float t64 = t5*vn;
            float t55 = t53+t54-t64;
            float t56 = t23-t29;
            float t65 = Tbs_flow.a.y*t56;
            float t57 = t28+t31-t65;
            float t58 = t26-t36;
            float t66 = Tbs_flow.a.z*t58;
            float t59 = t33+t35-t66;
            float t61 = P[0][0]*t2*t46;
            float t67 = P[1][1]*t2*t49;
            float t68 = P[4][0]*t2*t57;
            float t69 = P[5][0]*t2*t59;
            float t70 = P[6][0]*t2*t42;
            float t71 = P[1][0]*t2*t49;
            float t72 = P[2][0]*t2*t52;
            float t73 = P[3][0]*t2*t55;
            float t74 = t61+t68+t69+t70+t71+t72+t73;
            float t75 = t2*t46*t74;
            float t76 = P[4][1]*t2*t57;
            float t77 = P[5][1]*t2*t59;
            float t78 = P[6][1]*t2*t42;
            float t79 = P[0][1]*t2*t46;
            float t80 = P[2][1]*t2*t52;
            float t81 = P[3][1]*t2*t55;
            float t82 = t67+t76+t77+t78+t79+t80+t81;
            float t83 = t2*t49*t82;
            float t84 = P[4][2]*t2*t57;
            float t85 = P[5][2]*t2*t59;
            float t86 = P[6][2]*t2*t42;
            float t87 = P[0][2]*t2*t46;
            float t88 = P[1][2]*t2*t49;
            float t89 = P[2][2]*t2*t52;
            float t90 = P[3][2]*t2*t55;
            float t91 = t84+t85+t86+t87+t88+t89+t90;
            float t92 = t2*t52*t91;
            float t93 = P[4][3]*t2*t57;
            float t94 = P[5][3]*t2*t59;
            float t95 = P[6][3]*t2*t42;
            float t96 = P[0][3]*t2*t46;
            float t97 = P[1][3]*t2*t49;
            float t98 = P[2][3]*t2*t52;
            float t99 = P[3][3]*t2*t55;
            float t100 = t93+t94+t95+t96+t97+t98+t99;
            float t101 = t2*t55*t100;
            float t102 = P[4][4]*t2*t57;
            float t103 = P[5][4]*t2*t59;
            float t104 = P[6][4]*t2*t42;
            float t105 = P[0][4]*t2*t46;
            float t106 = P[1][4]*t2*t49;
            float t107 = P[2][4]*t2*t52;
            float t108 = P[3][4]*t2*t55;
            float t109 = t102+t103+t104+t105+t106+t107+t108;
            float t110 = t2*t57*t109;
            float t111 = P[4][5]*t2*t57;
            float t112 = P[5][5]*t2*t59;
            float t113 = P[6][5]*t2*t42;
            float t114 = P[0][5]*t2*t46;
            float t115 = P[1][5]*t2*t49;
            float t116 = P[2][5]*t2*t52;
            float t117 = P[3][5]*t2*t55;
            float t118 = t111+t112+t113+t114+t115+t116+t117;
            float t119 = t2*t59*t118;
            float t120 = P[4][6]*t2*t57;
            float t121 = P[5][6]*t2*t59;
            float t122 = P[6][6]*t2*t42;
            float t123 = P[0][6]*t2*t46;
            float t124 = P[1][6]*t2*t49;
            float t125 = P[2][6]*t2*t52;
            float t126 = P[3][6]*t2*t55;
            float t127 = t120+t121+t122+t123+t124+t125+t126;
            float t128 = t2*t42*t127;
            float t129 = R_LOS+t75+t83+t92+t101+t110+t119+t128;
            float t130 = 1.0f/t129;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t130 > R_LOS) {
                t130 = 1.0f/t129;
                faultStatus.bad_yflow = false;
            } else {
                t129 = R_LOS;
                t130 = 1.0f/R_LOS;
                faultStatus.bad_yflow = true;
                return;
            }
            varInnovOptFlow[1] = t129;

            H_LOS[0] = -t2*t46;
            H_LOS[1] = -t2*t49;
            H_LOS[2] = -t2*t52;
            H_LOS[3] = -t2*t55;
            H_LOS[4] = -t2*(t28+t31-Tbs_flow.a.y*(t23-q1*q2*2.0f));
            H_LOS[5] = -t2*(t33+t35-Tbs_flow.a.z*(t26-q2*q3*2.0f));
            H_LOS[6] = -t2*t42;

            // calculate innovation for Y observation
            innovOptFlow[1] = losPred[1] - ofDataDelayed.flowRadXYcomp.y;

            // calculate Kalman gains for the Y-axis observation
            Kfusion[0] = -t130*(t61+P[0][6]*t2*t42+P[0][1]*t2*t49+P[0][2]*t2*t52+P[0][3]*t2*t55+P[0][4]*t2*t57+P[0][5]*t2*t59);
            Kfusion[1] = -t130*(t67+P[1][0]*t2*t46+P[1][6]*t2*t42+P[1][2]*t2*t52+P[1][3]*t2*t55+P[1][4]*t2*t57+P[1][5]*t2*t59);
            Kfusion[2] = -t130*(t89+P[2][0]*t2*t46+P[2][6]*t2*t42+P[2][1]*t2*t49+P[2][3]*t2*t55+P[2][4]*t2*t57+P[2][5]*t2*t59);
            Kfusion[3] = -t130*(t99+P[3][0]*t2*t46+P[3][6]*t2*t42+P[3][1]*t2*t49+P[3][2]*t2*t52+P[3][4]*t2*t57+P[3][5]*t2*t59);
            Kfusion[4] = -t130*(t102+P[4][0]*t2*t46+P[4][6]*t2*t42+P[4][1]*t2*t49+P[4][2]*t2*t52+P[4][3]*t2*t55+P[4][5]*t2*t59);
            Kfusion[5] = -t130*(t112+P[5][0]*t2*t46+P[5][6]*t2*t42+P[5][1]*t2*t49+P[5][2]*t2*t52+P[5][3]*t2*t55+P[5][4]*t2*t57);
            Kfusion[6] = -t130*(t122+P[6][0]*t2*t46+P[6][1]*t2*t49+P[6][2]*t2*t52+P[6][3]*t2*t55+P[6][4]*t2*t57+P[6][5]*t2*t59);
            Kfusion[7] = -t130*(P[7][0]*t2*t46+P[7][6]*t2*t42+P[7][1]*t2*t49+P[7][2]*t2*t52+P[7][3]*t2*t55+P[7][4]*t2*t57+P[7][5]*t2*t59);
            Kfusion[8] = -t130*(P[8][0]*t2*t46+P[8][6]*t2*t42+P[8][1]*t2*t49+P[8][2]*t2*t52+P[8][3]*t2*t55+P[8][4]*t2*t57+P[8][5]*t2*t59);
            Kfusion[9] = -t130*(P[9][0]*t2*t46+P[9][6]*t2*t42+P[9][1]*t2*t49+P[9][2]*t2*t52+P[9][3]*t2*t55+P[9][4]*t2*t57+P[9][5]*t2*t59);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = -t130*(P[10][0]*t2*t46+P[10][6]*t2*t42+P[10][1]*t2*t49+P[10][2]*t2*t52+P[10][3]*t2*t55+P[10][4]*t2*t57+P[10][5]*t2*t59);
                Kfusion[11] = -t130*(P[11][0]*t2*t46+P[11][6]*t2*t42+P[11][1]*t2*t49+P[11][2]*t2*t52+P[11][3]*t2*t55+P[11][4]*t2*t57+P[11][5]*t2*t59);
                Kfusion[12] = -t130*(P[12][0]*t2*t46+P[12][6]*t2*t42+P[12][1]*t2*t49+P[12][2]*t2*t52+P[12][3]*t2*t55+P[12][4]*t2*t57+P[12][5]*t2*t59);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = -t130*(P[13][0]*t2*t46+P[13][6]*t2*t42+P[13][1]*t2*t49+P[13][2]*t2*t52+P[13][3]*t2*t55+P[13][4]*t2*t57+P[13][5]*t2*t59);
                Kfusion[14] = -t130*(P[14][0]*t2*t46+P[14][6]*t2*t42+P[14][1]*t2*t49+P[14][2]*t2*t52+P[14][3]*t2*t55+P[14][4]*t2*t57+P[14][5]*t2*t59);
                Kfusion[15] = -t130*(P[15][0]*t2*t46+P[15][6]*t2*t42+P[15][1]*t2*t49+P[15][2]*t2*t52+P[15][3]*t2*t55+P[15][4]*t2*t57+P[15][5]*t2*t59);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = -t130*(P[16][0]*t2*t46+P[16][6]*t2*t42+P[16][1]*t2*t49+P[16][2]*t2*t52+P[16][3]*t2*t55+P[16][4]*t2*t57+P[16][5]*t2*t59);
                Kfusion[17] = -t130*(P[17][0]*t2*t46+P[17][6]*t2*t42+P[17][1]*t2*t49+P[17][2]*t2*t52+P[17][3]*t2*t55+P[17][4]*t2*t57+P[17][5]*t2*t59);
                Kfusion[18] = -t130*(P[18][0]*t2*t46+P[18][6]*t2*t42+P[18][1]*t2*t49+P[18][2]*t2*t52+P[18][3]*t2*t55+P[18][4]*t2*t57+P[18][5]*t2*t59);
                Kfusion[19] = -t130*(P[19][0]*t2*t46+P[19][6]*t2*t42+P[19][1]*t2*t49+P[19][2]*t2*t52+P[19][3]*t2*t55+P[19][4]*t2*t57+P[19][5]*t2*t59);
                Kfusion[20] = -t130*(P[20][0]*t2*t46+P[20][6]*t2*t42+P[20][1]*t2*t49+P[20][2]*t2*t52+P[20][3]*t2*t55+P[20][4]*t2*t57+P[20][5]*t2*t59);
                Kfusion[21] = -t130*(P[21][0]*t2*t46+P[21][6]*t2*t42+P[21][1]*t2*t49+P[21][2]*t2*t52+P[21][3]*t2*t55+P[21][4]*t2*t57+P[21][5]*t2*t59);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = -t130*(P[22][0]*t2*t46+P[22][6]*t2*t42+P[22][1]*t2*t49+P[22][2]*t2*t52+P[22][3]*t2*t55+P[22][4]*t2*t57+P[22][5]*t2*t59);
                Kfusion[23] = -t130*(P[23][0]*t2*t46+P[23][6]*t2*t42+P[23][1]*t2*t49+P[23][2]*t2*t52+P[23][3]*t2*t55+P[23][4]*t2*t57+P[23][5]*t2*t59);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }
        }

        // calculate the innovation consistency test ratio
        flowTestRatio[obsIndex] = sq(innovOptFlow[obsIndex]) / (sq(MAX(0.01f * (float)frontend->_flowInnovGate, 1.0f)) * varInnovOptFlow[obsIndex]);

        // Check the innovation for consistency and don't fuse if out of bounds or flow is too fast to be reliable
        if ((flowTestRatio[obsIndex]) < 1.0f && (ofDataDelayed.flowRadXY.x < frontend->_maxFlowRate) && (ofDataDelayed.flowRadXY.y < frontend->_maxFlowRate)) {
            // record the last time observations were accepted for fusion
            prevFlowFuseTime_ms = imuSampleTime_ms;
            // notify first time only
            if (!flowFusionActive) {
                flowFusionActive = true;
                gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u fusing optical flow",(unsigned)imu_index);
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=6; j++) {
                    KH[i][j] = Kfusion[i] * H_LOS[j];
                }
                for (unsigned j = 7; j<=stateIndexLim; j++) {
                    KH[i][j] = 0.0f;
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
                    KHP[i][j] = res;
                }
            }

            // Check that we are not going to drive any variances negative and skip the update if so
            bool healthyFusion = true;
            for (uint8_t i= 0; i<=stateIndexLim; i++) {
                if (KHP[i][i] > P[i][i]) {
                    healthyFusion = false;
                }
            }

            if (healthyFusion) {
                // update the covariance matrix
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++) {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }

                // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
                ForceSymmetry();
                ConstrainVariances();

                // correct the state vector
                for (uint8_t j= 0; j<=stateIndexLim; j++) {
                    statesArray[j] = statesArray[j] - Kfusion[j] * innovOptFlow[obsIndex];
                }
                stateStruct.quat.normalize();

            } else {
                // record bad axis
                if (obsIndex == 0) {
                    faultStatus.bad_xflow = true;
                } else if (obsIndex == 1) {
                    faultStatus.bad_yflow = true;
                }

            }
        }
    }
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

#endif // HAL_CPU_CLASS
