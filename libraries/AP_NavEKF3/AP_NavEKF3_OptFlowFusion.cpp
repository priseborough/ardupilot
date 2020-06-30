#include <AP_HAL/AP_HAL.h>

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

    // if have valid flow or range measurements, fuse data into a 1-state EKF to estimate terrain height
    if (((flowDataToFuse && (frontend->_flowUse == FLOW_USE_TERRAIN)) || rangeDataToFuse) && tiltOK) {
        // Estimate the terrain offset (runs a one state EKF)
        EstimateTerrainOffset();
    }

    // Fuse optical flow data into the main filter
    if (flowDataToFuse && tiltOK) {
        if (frontend->_flowUse == FLOW_USE_NAV) {
            // Set the flow noise used by the fusion processes
            R_LOS = sq(MAX(frontend->_flowNoise, 0.05f));
            // Fuse the optical flow X and Y axis data into the main filter sequentially
            FuseOptFlow();
        }
        // reset flag to indicate that no new flow data is available for fusion
        flowDataToFuse = false;
    }

    // stop the performance timer
    hal.util->perf_end(_perf_FuseOptFlow);
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optical flow rates and range finder measurements
Equations generated using https://github.com/PX4/ecl/tree/master/EKF/matlab/scripts/Terrain%20Estimator
*/
void NavEKF3_core::EstimateTerrainOffset()
{
    // start performance timer
    hal.util->perf_begin(_perf_TerrainOffset);

    // horizontal velocity squared
    float velHorizSq = sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y);

    // don't fuse flow data if LOS rate is misaligned, without GPS, or insufficient velocity, as it is poorly observable
    // don't fuse flow data if it exceeds validity limits
    // don't update terrain offset if ground is being used as the zero height datum in the main filter
    bool cantFuseFlowData = ((frontend->_flowUse != FLOW_USE_TERRAIN)
    || gpsNotAvailable 
    || PV_AidingMode == AID_RELATIVE 
    || velHorizSq < 25.0f 
    || (MAX(ofDataDelayed.flowRadXY[0],ofDataDelayed.flowRadXY[1]) > frontend->_maxFlowRate));

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
        float Pincrement = (distanceTravelledSq * sq(frontend->_terrGradMax)) + sq(timeLapsed)*P[6][6];
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
            const float SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
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

        if (!cantFuseFlowData) {

            Vector3f relVelSensor;          // velocity of sensor relative to ground in sensor axes
            Vector2f losPred;               // predicted optical flow angular rate measurement
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time
            float K_OPT;
            float H_OPT;
            Vector2f auxFlowObsInnovVar;

            // predict range to centre of image
            float flowRngPred = MAX((terrainState - stateStruct.position.z),rngOnGnd) / prevTnb.c.z;

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

            // calculate relative velocity in sensor frame
            relVelSensor = prevTnb*stateStruct.velocity;

            // divide velocity by range, subtract body rates and apply scale factor to
            // get predicted sensed angular optical rates relative to X and Y sensor axes
            losPred.x =   relVelSensor.y / flowRngPred;
            losPred.y = - relVelSensor.x / flowRngPred;

            // calculate innovations
            auxFlowObsInnov = losPred - ofDataDelayed.flowRadXYcomp;

            // calculate observation jacobians 
            float t2 = q0*q0;
            float t3 = q1*q1;
            float t4 = q2*q2;
            float t5 = q3*q3;
            float t6 = stateStruct.position.z - terrainState;
            float t7 = 1.0f / (t6*t6);
            float t8 = q0*q3*2.0f;
            float t9 = t2-t3-t4+t5;

            // prevent the state variances from becoming badly conditioned
            Popt = MAX(Popt,1E-6f);

            // calculate observation noise variance from parameter
            float flow_noise_variance = sq(MAX(frontend->_flowNoise, 0.05f));

            // Fuse Y axis data

            // Calculate observation partial derivative
            H_OPT = t7*t9*(-stateStruct.velocity.z*(q0*q2*2.0-q1*q3*2.0)+stateStruct.velocity.x*(t2+t3-t4-t5)+stateStruct.velocity.y*(t8+q1*q2*2.0));

            // calculate innovation variance
            auxFlowObsInnovVar.y = H_OPT * Popt * H_OPT + flow_noise_variance;

            // calculate Kalman gain
            K_OPT = Popt * H_OPT / auxFlowObsInnovVar.y;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio.y = sq(auxFlowObsInnov.y) / (sq(MAX(0.01f * (float)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar.y);

            // don't fuse if optical flow data is outside valid range
            if (auxFlowTestRatio.y < 1.0f) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov.y;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

                // update intermediate variables used when fusing the X axis
                t6 = stateStruct.position.z - terrainState;
                t7 = 1.0f / (t6*t6);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming badly conditioned
                Popt = MAX(Popt,1E-6f);
            }

            // fuse X axis data
            H_OPT = -t7*t9*(stateStruct.velocity.z*(q0*q1*2.0+q2*q3*2.0)+stateStruct.velocity.y*(t2-t3+t4-t5)-stateStruct.velocity.x*(t8-q1*q2*2.0));

            // calculate innovation variances
            auxFlowObsInnovVar.x = H_OPT * Popt * H_OPT + flow_noise_variance;

            // calculate Kalman gain
            K_OPT = Popt * H_OPT / auxFlowObsInnovVar.x;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio.x = sq(auxFlowObsInnov.x) / (sq(MAX(0.01f * (float)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar.x);

            // don't fuse if optical flow data is outside valid range
            if (auxFlowTestRatio.x < 1.0f) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov.x;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming badly conditioned
                Popt = MAX(Popt,1E-6f);
            }
        }
    }

    // stop the performance timer
    hal.util->perf_end(_perf_TerrainOffset);
}

/*
 * Fuse angular motion compensated optical flow rates using explicit algebraic equations generated
 * from python/main.py sympy script
 * Requires a valid terrain height estimate.
*/
void NavEKF3_core::FuseOptFlow()
{
    Vector24 H_LOS;
    Vector3f relVelSensor;
    Vector2 losPred;

    // Copy required states to local variable names
    const float &q0  = stateStruct.quat[0];
    const float &q1 = stateStruct.quat[1];
    const float &q2 = stateStruct.quat[2];
    const float &q3 = stateStruct.quat[3];
    const float &vn = stateStruct.velocity.x;
    const float &ve = stateStruct.velocity.y;
    const float &vd = stateStruct.velocity.z;
    const float &pd = stateStruct.position.z;

    // The derivation allows for a body to flow sensor frame rotation matrix which is
    // currently not supported by the EKF so assume sensor frame is aligned with the
    // body frame
    const float Tbs[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

    // constrain height above ground to be above range measured on ground
    float heightAboveGndEst = MAX((terrainState - pd), rngOnGnd);

    // Fuse X and Y axis measurements sequentially assuming observation errors are uncorrelated
    for (uint8_t obsIndex=0; obsIndex<=1; obsIndex++) { // fuse X axis data first
        // calculate range from ground plain to centre of sensor fov assuming flat earth
        float range = constrain_float((heightAboveGndEst/prevTnb.c.z),rngOnGnd,1000.0f);

        // correct range for flow sensor offset body frame position offset
        // the corrected value is the predicted range from the sensor focal point to the
        // centre of the image on the ground assuming flat terrain
        Vector3f posOffsetBody = (*ofDataDelayed.body_offset) - accelPosOffset;
        if (!posOffsetBody.is_zero()) {
            Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
            range -= posOffsetEarth.z / prevTnb.c.z;
        }

        // calculate relative velocity in sensor frame including the relative motion due to rotation
        relVelSensor = (prevTnb * stateStruct.velocity) + (ofDataDelayed.bodyRadXYZ % posOffsetBody);

        // divide velocity by range to get predicted angular LOS rates relative to X and Y axes
        range = MAX(range, 0.01f);
        losPred[0] =  relVelSensor.y/range;
        losPred[1] = -relVelSensor.x/range;

        // calculate observation jacobians and Kalman gains
        memset(&H_LOS[0], 0, sizeof(H_LOS));
        if (obsIndex == 0) {
            // Calculate X axis observation Jacobian, innovation variance and Kalman gain.
            const float SX0 = -Tbs[1][0]*q2 + Tbs[1][1]*q1 + Tbs[1][2]*q0;
            const float SX1 = Tbs[1][0]*q3 + Tbs[1][1]*q0 - Tbs[1][2]*q1;
            const float SX2 = Tbs[1][0]*q0 - Tbs[1][1]*q3 + Tbs[1][2]*q2;
            const float SX3 = SX0*vd + SX1*ve + SX2*vn;
            const float SX4 = 1.0F/range;
            const float SX5 = 2*SX4;
            const float SX6 = Tbs[1][0]*q1 + Tbs[1][1]*q2 + Tbs[1][2]*q3;
            const float SX7 = -SX0*ve + SX1*vd + SX6*vn;
            const float SX8 = SX0*vn - SX2*vd + SX6*ve;
            const float SX9 = -SX1*vn + SX2*ve + SX6*vd;
            const float SX10 = q0*q2;
            const float SX11 = q1*q3;
            const float SX12 = 2*Tbs[1][2];
            const float SX13 = q0*q3;
            const float SX14 = q1*q2;
            const float SX15 = 2*Tbs[1][1];
            const float SX16 = powf(q1, 2);
            const float SX17 = powf(q2, 2);
            const float SX18 = -SX17;
            const float SX19 = powf(q0, 2);
            const float SX20 = powf(q3, 2);
            const float SX21 = SX19 - SX20;
            const float SX22 = SX12*(SX10 + SX11) - SX15*(SX13 - SX14) + Tbs[1][0]*(SX16 + SX18 + SX21);
            const float SX23 = 2*Tbs[1][0];
            const float SX24 = q0*q1;
            const float SX25 = q2*q3;
            const float SX26 = -SX16;
            const float SX27 = -SX12*(SX24 - SX25) + SX23*(SX13 + SX14) + Tbs[1][1]*(SX17 + SX21 + SX26);
            const float SX28 = SX15*(SX24 + SX25) - SX23*(SX10 - SX11) + Tbs[1][2]*(SX18 + SX19 + SX20 + SX26);
            const float SX29 = 2*SX3;
            const float SX30 = 2*SX7;
            const float SX31 = 2*SX8;
            const float SX32 = 2*SX9;
            const float SX33 = P[0][0]*SX29 + P[0][1]*SX30 + P[0][2]*SX31 + P[0][3]*SX32 + P[0][4]*SX22 + P[0][5]*SX27 + P[0][6]*SX28;
            const float SX34 = powf(range, -2);
            const float SX35 = P[0][6]*SX29 + P[1][6]*SX30 + P[2][6]*SX31 + P[3][6]*SX32 + P[4][6]*SX22 + P[5][6]*SX27 + P[6][6]*SX28;
            const float SX36 = P[0][5]*SX29 + P[1][5]*SX30 + P[2][5]*SX31 + P[3][5]*SX32 + P[4][5]*SX22 + P[5][5]*SX27 + P[5][6]*SX28;
            const float SX37 = P[0][4]*SX29 + P[1][4]*SX30 + P[2][4]*SX31 + P[3][4]*SX32 + P[4][4]*SX22 + P[4][5]*SX27 + P[4][6]*SX28;
            const float SX38 = P[0][2]*SX29 + P[1][2]*SX30 + P[2][2]*SX31 + P[2][3]*SX32 + P[2][4]*SX22 + P[2][5]*SX27 + P[2][6]*SX28;
            const float SX39 = P[0][3]*SX29 + P[1][3]*SX30 + P[2][3]*SX31 + P[3][3]*SX32 + P[3][4]*SX22 + P[3][5]*SX27 + P[3][6]*SX28;
            const float SX40 = P[0][1]*SX29 + P[1][1]*SX30 + P[1][2]*SX31 + P[1][3]*SX32 + P[1][4]*SX22 + P[1][5]*SX27 + P[1][6]*SX28;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            varInnovOptFlow[0] = (R_LOS + SX22*SX34*SX37 + SX27*SX34*SX36 + SX28*SX34*SX35 + SX29*SX33*SX34 + SX30*SX34*SX40 + SX31*SX34*SX38 + SX32*SX34*SX39);
            float SX41;
            if (varInnovOptFlow[0] > R_LOS) {
                SX41 = SX4 / varInnovOptFlow[0];
                faultStatus.bad_xflow = false;
            } else {
                faultStatus.bad_xflow = true;
                return;
            }

            // observation Jacobians
            H_LOS[0] = SX3*SX5;
            H_LOS[1] = SX5*SX7;
            H_LOS[2] = SX5*SX8;
            H_LOS[3] = SX5*SX9;
            H_LOS[4] = SX22*SX4;
            H_LOS[5] = SX27*SX4;
            H_LOS[6] = SX28*SX4;

            // calculate innovation for X axis observation
            innovOptFlow[0] = losPred[0] - ofDataDelayed.flowRadXYcomp.x;

            // calculate Kalman gains for X-axis observation
            Kfusion[0] = SX33*SX41;
            Kfusion[1] = SX40*SX41;
            Kfusion[2] = SX38*SX41;
            Kfusion[3] = SX39*SX41;
            Kfusion[4] = SX37*SX41;
            Kfusion[5] = SX36*SX41;
            Kfusion[6] = SX35*SX41;
            Kfusion[7] = SX41*(P[0][7]*SX29 + P[1][7]*SX30 + P[2][7]*SX31 + P[3][7]*SX32 + P[4][7]*SX22 + P[5][7]*SX27 + P[6][7]*SX28);
            Kfusion[8] = SX41*(P[0][8]*SX29 + P[1][8]*SX30 + P[2][8]*SX31 + P[3][8]*SX32 + P[4][8]*SX22 + P[5][8]*SX27 + P[6][8]*SX28);
            Kfusion[9] = SX41*(P[0][9]*SX29 + P[1][9]*SX30 + P[2][9]*SX31 + P[3][9]*SX32 + P[4][9]*SX22 + P[5][9]*SX27 + P[6][9]*SX28);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = SX41*(P[0][10]*SX29 + P[1][10]*SX30 + P[2][10]*SX31 + P[3][10]*SX32 + P[4][10]*SX22 + P[5][10]*SX27 + P[6][10]*SX28);
                Kfusion[11] = SX41*(P[0][11]*SX29 + P[1][11]*SX30 + P[2][11]*SX31 + P[3][11]*SX32 + P[4][11]*SX22 + P[5][11]*SX27 + P[6][11]*SX28);
                Kfusion[12] = SX41*(P[0][12]*SX29 + P[1][12]*SX30 + P[2][12]*SX31 + P[3][12]*SX32 + P[4][12]*SX22 + P[5][12]*SX27 + P[6][12]*SX28);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = SX41*(P[0][13]*SX29 + P[1][13]*SX30 + P[2][13]*SX31 + P[3][13]*SX32 + P[4][13]*SX22 + P[5][13]*SX27 + P[6][13]*SX28);
                Kfusion[14] = SX41*(P[0][14]*SX29 + P[1][14]*SX30 + P[2][14]*SX31 + P[3][14]*SX32 + P[4][14]*SX22 + P[5][14]*SX27 + P[6][14]*SX28);
                Kfusion[15] = SX41*(P[0][15]*SX29 + P[1][15]*SX30 + P[2][15]*SX31 + P[3][15]*SX32 + P[4][15]*SX22 + P[5][15]*SX27 + P[6][15]*SX28);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = SX41*(P[0][16]*SX29 + P[1][16]*SX30 + P[2][16]*SX31 + P[3][16]*SX32 + P[4][16]*SX22 + P[5][16]*SX27 + P[6][16]*SX28);
                Kfusion[17] = SX41*(P[0][17]*SX29 + P[1][17]*SX30 + P[2][17]*SX31 + P[3][17]*SX32 + P[4][17]*SX22 + P[5][17]*SX27 + P[6][17]*SX28);
                Kfusion[18] = SX41*(P[0][18]*SX29 + P[1][18]*SX30 + P[2][18]*SX31 + P[3][18]*SX32 + P[4][18]*SX22 + P[5][18]*SX27 + P[6][18]*SX28);
                Kfusion[19] = SX41*(P[0][19]*SX29 + P[1][19]*SX30 + P[2][19]*SX31 + P[3][19]*SX32 + P[4][19]*SX22 + P[5][19]*SX27 + P[6][19]*SX28);
                Kfusion[20] = SX41*(P[0][20]*SX29 + P[1][20]*SX30 + P[2][20]*SX31 + P[3][20]*SX32 + P[4][20]*SX22 + P[5][20]*SX27 + P[6][20]*SX28);
                Kfusion[21] = SX41*(P[0][21]*SX29 + P[1][21]*SX30 + P[2][21]*SX31 + P[3][21]*SX32 + P[4][21]*SX22 + P[5][21]*SX27 + P[6][21]*SX28);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = SX41*(P[0][22]*SX29 + P[1][22]*SX30 + P[2][22]*SX31 + P[3][22]*SX32 + P[4][22]*SX22 + P[5][22]*SX27 + P[6][22]*SX28);
                Kfusion[23] = SX41*(P[0][23]*SX29 + P[1][23]*SX30 + P[2][23]*SX31 + P[3][23]*SX32 + P[4][23]*SX22 + P[5][23]*SX27 + P[6][23]*SX28);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }

        } else {

            // calculate X axis observation Jacobian, innovation variance and Kalman gain
            const float SY0 = Tbs[0][1]*q1;
            const float SY1 = Tbs[0][2]*q0;
            const float SY2 = Tbs[0][0]*q2;
            const float SY3 = SY0 + SY1 - SY2;
            const float SY4 = Tbs[0][0]*q3;
            const float SY5 = Tbs[0][1]*q0;
            const float SY6 = Tbs[0][2]*q1;
            const float SY7 = SY4 + SY5 - SY6;
            const float SY8 = Tbs[0][0]*q0;
            const float SY9 = Tbs[0][2]*q2;
            const float SY10 = Tbs[0][1]*q3;
            const float SY11 = -SY10 + SY8 + SY9;
            const float SY12 = SY11*vn + SY3*vd + SY7*ve;
            const float SY13 = 1.0F/range;
            const float SY14 = 2*SY13;
            const float SY15 = Tbs[0][0]*q1 + Tbs[0][1]*q2 + Tbs[0][2]*q3;
            const float SY16 = SY15*vn + SY7*vd;
            const float SY17 = SY16 - SY3*ve;
            const float SY18 = SY15*ve + SY3*vn;
            const float SY19 = -SY11*vd + SY18;
            const float SY20 = SY11*ve + SY15*vd;
            const float SY21 = SY20 - SY7*vn;
            const float SY22 = q0*q3;
            const float SY23 = q1*q2;
            const float SY24 = 2*Tbs[0][1];
            const float SY25 = powf(q1, 2);
            const float SY26 = powf(q2, 2);
            const float SY27 = -SY26;
            const float SY28 = powf(q0, 2);
            const float SY29 = powf(q3, 2);
            const float SY30 = SY28 - SY29;
            const float SY31 = q0*q2;
            const float SY32 = q1*q3;
            const float SY33 = 2*Tbs[0][2];
            const float SY34 = SY33*(SY31 + SY32) + Tbs[0][0]*(SY25 + SY27 + SY30);
            const float SY35 = -SY24*(SY22 - SY23) + SY34;
            const float SY36 = q0*q1;
            const float SY37 = q2*q3;
            const float SY38 = -SY25;
            const float SY39 = 2*Tbs[0][0];
            const float SY40 = SY39*(SY22 + SY23) + Tbs[0][1]*(SY26 + SY30 + SY38);
            const float SY41 = -SY33*(SY36 - SY37) + SY40;
            const float SY42 = SY24*(SY36 + SY37) + Tbs[0][2]*(SY27 + SY28 + SY29 + SY38);
            const float SY43 = -SY39*(SY31 - SY32) + SY42;
            const float SY44 = 2*SY12;
            const float SY45 = 2*SY16 + 2*ve*(-SY0 - SY1 + SY2);
            const float SY46 = 2*SY18 + 2*vd*(SY10 - SY8 - SY9);
            const float SY47 = 2*SY20 + 2*vn*(-SY4 - SY5 + SY6);
            const float SY48 = SY24*(-SY22 + SY23) + SY34;
            const float SY49 = SY33*(-SY36 + SY37) + SY40;
            const float SY50 = SY39*(-SY31 + SY32) + SY42;
            const float SY51 = P[0][0]*SY44 + P[0][1]*SY45 + P[0][2]*SY46 + P[0][3]*SY47 + P[0][4]*SY48 + P[0][5]*SY49 + P[0][6]*SY50;
            const float SY52 = powf(range, -2);
            const float SY53 = P[0][6]*SY44 + P[1][6]*SY45 + P[2][6]*SY46 + P[3][6]*SY47 + P[4][6]*SY48 + P[5][6]*SY49 + P[6][6]*SY50;
            const float SY54 = P[0][5]*SY44 + P[1][5]*SY45 + P[2][5]*SY46 + P[3][5]*SY47 + P[4][5]*SY48 + P[5][5]*SY49 + P[5][6]*SY50;
            const float SY55 = P[0][4]*SY44 + P[1][4]*SY45 + P[2][4]*SY46 + P[3][4]*SY47 + P[4][4]*SY48 + P[4][5]*SY49 + P[4][6]*SY50;
            const float SY56 = P[0][2]*SY44 + P[1][2]*SY45 + P[2][2]*SY46 + P[2][3]*SY47 + P[2][4]*SY48 + P[2][5]*SY49 + P[2][6]*SY50;
            const float SY57 = 2*SY52;
            const float SY58 = P[0][3]*SY44 + P[1][3]*SY45 + P[2][3]*SY46 + P[3][3]*SY47 + P[3][4]*SY48 + P[3][5]*SY49 + P[3][6]*SY50;
            const float SY59 = P[0][1]*SY44 + P[1][1]*SY45 + P[1][2]*SY46 + P[1][3]*SY47 + P[1][4]*SY48 + P[1][5]*SY49 + P[1][6]*SY50;

            // calculate innovation variance for Y axis observation and protect against a badly conditioned calculation
            varInnovOptFlow[1] = (R_LOS + SY17*SY57*SY59 + SY19*SY56*SY57 + SY21*SY57*SY58 + SY35*SY52*SY55 + SY41*SY52*SY54 + SY43*SY52*SY53 + SY44*SY51*SY52);
            float SY60;
            if (varInnovOptFlow[1] > R_LOS) {
                SY60 = SY13 / varInnovOptFlow[1];
                faultStatus.bad_yflow = false;
            } else {
                faultStatus.bad_yflow = true;
                return;
            }

            // observation Jacobians
            H_LOS[0] = -SY12*SY14;
            H_LOS[1] = -SY14*SY17;
            H_LOS[2] = -SY14*SY19;
            H_LOS[3] = -SY14*SY21;
            H_LOS[4] = -SY13*SY35;
            H_LOS[5] = -SY13*SY41;
            H_LOS[6] = -SY13*SY43;

            // calculate innovation for Y observation
            innovOptFlow[1] = losPred[1] - ofDataDelayed.flowRadXYcomp.y;

            // calculate Kalman gains for the Y-axis observation
            Kfusion[0] = -SY51*SY60;
            Kfusion[1] = -SY59*SY60;
            Kfusion[2] = -SY56*SY60;
            Kfusion[3] = -SY58*SY60;
            Kfusion[4] = -SY55*SY60;
            Kfusion[5] = -SY54*SY60;
            Kfusion[6] = -SY53*SY60;
            Kfusion[7] = -SY60*(P[0][7]*SY44 + P[1][7]*SY45 + P[2][7]*SY46 + P[3][7]*SY47 + P[4][7]*SY48 + P[5][7]*SY49 + P[6][7]*SY50);
            Kfusion[8] = -SY60*(P[0][8]*SY44 + P[1][8]*SY45 + P[2][8]*SY46 + P[3][8]*SY47 + P[4][8]*SY48 + P[5][8]*SY49 + P[6][8]*SY50);
            Kfusion[9] = -SY60*(P[0][9]*SY44 + P[1][9]*SY45 + P[2][9]*SY46 + P[3][9]*SY47 + P[4][9]*SY48 + P[5][9]*SY49 + P[6][9]*SY50);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = -SY60*(P[0][10]*SY44 + P[1][10]*SY45 + P[2][10]*SY46 + P[3][10]*SY47 + P[4][10]*SY48 + P[5][10]*SY49 + P[6][10]*SY50);
                Kfusion[11] = -SY60*(P[0][11]*SY44 + P[1][11]*SY45 + P[2][11]*SY46 + P[3][11]*SY47 + P[4][11]*SY48 + P[5][11]*SY49 + P[6][11]*SY50);
                Kfusion[12] = -SY60*(P[0][12]*SY44 + P[1][12]*SY45 + P[2][12]*SY46 + P[3][12]*SY47 + P[4][12]*SY48 + P[5][12]*SY49 + P[6][12]*SY50);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = -SY60*(P[0][13]*SY44 + P[1][13]*SY45 + P[2][13]*SY46 + P[3][13]*SY47 + P[4][13]*SY48 + P[5][13]*SY49 + P[6][13]*SY50);
                Kfusion[14] = -SY60*(P[0][14]*SY44 + P[1][14]*SY45 + P[2][14]*SY46 + P[3][14]*SY47 + P[4][14]*SY48 + P[5][14]*SY49 + P[6][14]*SY50);
                Kfusion[15] = -SY60*(P[0][15]*SY44 + P[1][15]*SY45 + P[2][15]*SY46 + P[3][15]*SY47 + P[4][15]*SY48 + P[5][15]*SY49 + P[6][15]*SY50);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = -SY60*(P[0][16]*SY44 + P[1][16]*SY45 + P[2][16]*SY46 + P[3][16]*SY47 + P[4][16]*SY48 + P[5][16]*SY49 + P[6][16]*SY50);
                Kfusion[17] = -SY60*(P[0][17]*SY44 + P[1][17]*SY45 + P[2][17]*SY46 + P[3][17]*SY47 + P[4][17]*SY48 + P[5][17]*SY49 + P[6][17]*SY50);
                Kfusion[18] = -SY60*(P[0][18]*SY44 + P[1][18]*SY45 + P[2][18]*SY46 + P[3][18]*SY47 + P[4][18]*SY48 + P[5][18]*SY49 + P[6][18]*SY50);
                Kfusion[19] = -SY60*(P[0][19]*SY44 + P[1][19]*SY45 + P[2][19]*SY46 + P[3][19]*SY47 + P[4][19]*SY48 + P[5][19]*SY49 + P[6][19]*SY50);
                Kfusion[20] = -SY60*(P[0][20]*SY44 + P[1][20]*SY45 + P[2][20]*SY46 + P[3][20]*SY47 + P[4][20]*SY48 + P[5][20]*SY49 + P[6][20]*SY50);
                Kfusion[21] = -SY60*(P[0][21]*SY44 + P[1][21]*SY45 + P[2][21]*SY46 + P[3][21]*SY47 + P[4][21]*SY48 + P[5][21]*SY49 + P[6][21]*SY50);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = -SY60*(P[0][22]*SY44 + P[1][22]*SY45 + P[2][22]*SY46 + P[3][22]*SY47 + P[4][22]*SY48 + P[5][22]*SY49 + P[6][22]*SY50);
                Kfusion[23] = -SY60*(P[0][23]*SY44 + P[1][23]*SY45 + P[2][23]*SY46 + P[3][23]*SY47 + P[4][23]*SY48 + P[5][23]*SY49 + P[6][23]*SY50);
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

                // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
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

