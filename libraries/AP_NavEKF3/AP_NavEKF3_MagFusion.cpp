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

// Control reset of yaw and magnetic field states
void NavEKF3_core::controlMagYawReset()
{

    // Vehicles that can use a zero sideslip assumption (Planes) are a special case
    // They can use the GPS velocity to recover from bad initial compass data
    // This allows recovery for heading alignment errors due to compass faults
    if (assume_zero_sideslip() && !finalInflightYawInit && inFlight) {
        gpsYawResetRequest = true;
        return;
    } else {
        gpsYawResetRequest = false;
    }

    // Quaternion and delta rotation vector that are re-used for different calculations
    Vector3f deltaRotVecTemp;
    Quaternion deltaQuatTemp;

    bool flightResetAllowed = false;
    bool initialResetAllowed = false;
    if (!finalInflightYawInit) {
        // Use a quaternion division to calculate the delta quaternion between the rotation at the current and last time
        deltaQuatTemp = stateStruct.quat / prevQuatMagReset;
        prevQuatMagReset = stateStruct.quat;

        // convert the quaternion to a rotation vector and find its length
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);

        // check if the spin rate is OK - high spin rates can cause angular alignment errors
        bool angRateOK = deltaRotVecTemp.length() < 0.1745f;

        initialResetAllowed = angRateOK;
        flightResetAllowed = angRateOK && !onGround;

    }

    // reset the limit on the number of magnetic anomaly resets for each takeoff
    if (onGround) {
        magYawAnomallyCount = 0;
    }

    // Check if conditions for a interim or final yaw/mag reset are met
    bool finalResetRequest = false;
    bool interimResetRequest = false;
    if (flightResetAllowed && !assume_zero_sideslip()) {
        // check that we have reached a height where ground magnetic interference effects are insignificant
        // and can perform a final reset of the yaw and field states
        finalResetRequest = (stateStruct.position.z  - posDownAtTakeoff) < -EKF3_MAG_FINAL_RESET_ALT;

        // check for increasing height
        bool hgtIncreasing = (posDownAtLastMagReset-stateStruct.position.z) > 0.5f;
        float yawInnovIncrease = fabsf(innovYaw) - fabsf(yawInnovAtLastMagReset);

        // check for increasing yaw innovations
        bool yawInnovIncreasing = yawInnovIncrease > 0.25f;

        // check that the yaw innovations haven't been caused by a large change in attitude
        deltaQuatTemp = quatAtLastMagReset / stateStruct.quat;
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);
        bool largeAngleChange = deltaRotVecTemp.length() > yawInnovIncrease;

        // if yaw innovations and height have increased and we haven't rotated much
        // then we are climbing away from a ground based magnetic anomaly and need to reset
        interimResetRequest = !finalInflightYawInit
                                && !finalResetRequest
                                && (magYawAnomallyCount < MAG_ANOMALY_RESET_MAX)
                                && hgtIncreasing
                                && yawInnovIncreasing
                                && !largeAngleChange;
    }

    // an initial reset is required if we have not yet aligned the yaw angle
    bool initialResetRequest = initialResetAllowed && !yawAlignComplete;

    // a combined yaw angle and magnetic field reset can be initiated by:
    magYawResetRequest = magYawResetRequest || // an external request
            initialResetRequest || // an initial alignment performed by all vehicle types using magnetometer
            interimResetRequest || // an interim alignment required to recover from ground based magnetic anomaly
            finalResetRequest; // the final reset when we have achieved enough height to be in stable magnetic field environment

    // Perform a reset of magnetic field states and reset yaw to corrected magnetic heading
    if (magYawResetRequest) {
        // update rotation matrix from body to NED frame
        stateStruct.quat.inverse().rotation_matrix(prevTnb);

        // set yaw from a single mag sample
        setYawFromMag();

        // send initial alignment status to console
        if (!yawAlignComplete) {
            gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u initial yaw alignment complete",(unsigned)imu_index);
        }

        // send in-flight yaw alignment status to console
        if (finalResetRequest) {
            gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u in-flight yaw alignment complete",(unsigned)imu_index);
        } else if (interimResetRequest) {
            magYawAnomallyCount++;
            gcs().send_text(MAV_SEVERITY_WARNING, "EKF3 IMU%u ground mag anomaly, yaw re-aligned",(unsigned)imu_index);
        }

        // clear the complete flags if an interim reset has been performed to allow subsequent
        // and final reset to occur
        if (interimResetRequest) {
            finalInflightYawInit = false;
            finalInflightMagInit = false;
        }
    }

    if (magStateResetRequest) {

        // Rotate Mag measurements into NED to set initial NED magnetic field states
        // Don't do this if the earth field has already been learned
        if (!magFieldLearned) {
            if (have_table_earth_field && frontend->_mag_ef_limit > 0) {
                stateStruct.earth_magfield = table_earth_field_ga;
            } else {
                stateStruct.earth_magfield = prevTnb.transposed() * magDataDelayed.mag;
            }

            // set the NE earth magnetic field states using the published declination
            // and set the corresponding variances and covariances
            alignMagStateDeclination();

            // set the remaining variances and covariances
            zeroRows(P,18,21);
            zeroCols(P,18,21);
            P[18][18] = sq(frontend->_magNoise);
            P[19][19] = P[18][18];
            P[20][20] = P[18][18];
            P[21][21] = P[18][18];

        }

        // record the fact we have initialised the magnetic field states
        recordMagReset();

        // prevent reset of variances in ConstrainVariances()
        inhibitMagStates = false;

        // clear mag state reset request
        magStateResetRequest = false;

    }
}

// this function is used to do a forced re-alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF3_core::realignYawGPS()
{
    // get quaternion from existing filter states and calculate roll, pitch and yaw angles
    Vector3f eulerAngles;
    stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

    if ((sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y)) > 25.0f) {
        // calculate course yaw angle
        float velYaw = atan2f(stateStruct.velocity.y,stateStruct.velocity.x);

        // calculate course yaw angle from GPS velocity
        float gpsYaw = atan2f(gpsDataDelayed.vel.y,gpsDataDelayed.vel.x);

        // Check the yaw angles for consistency
        float yawErr = MAX(fabsf(wrap_PI(gpsYaw - velYaw)),fabsf(wrap_PI(gpsYaw - eulerAngles.z)));

        // If the angles disagree by more than 45 degrees and GPS innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
        badMagYaw = ((yawErr > 0.7854f) && (velTestRatio > 1.0f) && (PV_AidingMode == AID_ABSOLUTE)) || !yawAlignComplete;

        // correct yaw angle using GPS ground course if compass yaw bad
        if (badMagYaw) {
            // attempt to use EKF-GSF estimate if available as it is more robust to GPS glitches
            if (EKFGSF_resetMainFilterYaw()) {
                return;
            }

            // keep roll and pitch and reset yaw
            resetQuatStateYawOnly(gpsYaw, sq(radians(45.0f)));

            // reset the velocity and position states as they will be inaccurate due to bad yaw
            velResetSource = GPS;
            ResetVelocity();
            posResetSource = GPS;
            ResetPosition();

            // send yaw alignment information to console
            gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u yaw aligned to GPS velocity",(unsigned)imu_index);

            if (use_compass()) {
                // request a mag field reset which may enable us to use the magnetometer if the previous fault was due to bad initialisation
                magStateResetRequest = true;
                // clear the all sensors failed status so that the magnetometers sensors get a second chance now that we are flying
                allMagSensorsFailed = false;
            }
        }
    }
}

void NavEKF3_core::alignYawAngle()
{
    // calculate the variance for the rotation estimate expressed as a rotation vector
    // this will be used later to reset the quaternion state covariances
    Vector3f angleErrVarVec = calcRotVecVariances();

    if (yawAngDataDelayed.type == 2) {
        Vector3f euler321;
        stateStruct.quat.to_euler(euler321.x, euler321.y, euler321.z);
        stateStruct.quat.from_euler(euler321.x, euler321.y, yawAngDataDelayed.yawAng);
    } else if (yawAngDataDelayed.type == 1) {
        Vector3f euler312 = stateStruct.quat.to_vector312();
        stateStruct.quat.from_vector312(euler312.x, euler312.y, yawAngDataDelayed.yawAng);
    }

    // set the yaw angle variance to a larger value to reflect the uncertainty in yaw
    angleErrVarVec.z = sq(yawAngDataDelayed.yawAngErr);

    // reset the quaternion covariances using the rotation vector variances
    zeroRows(P,0,3);
    zeroCols(P,0,3);
    initialiseQuatCovariances(angleErrVarVec);

    // send yaw alignment information to console
    gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u yaw aligned",(unsigned)imu_index);

    // record the yaw reset event
    recordYawReset();

    // clear any pending yaw reset requests
    gpsYawResetRequest = false;
    magYawResetRequest = false;

}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of magnetometer data
void NavEKF3_core::SelectMagFusion()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseMagnetometer);

    // clear the flag that lets other processes know that the expensive magnetometer fusion operation has been performed on that time step
    // used for load levelling
    magFusePerformed = false;

    effectiveMagCal = effective_magCal();

    // Handle case where we are not using a yaw sensor of any type and and attempt to reset the yaw in
    // flight using the output from the GSF yaw estimator.
    if (!use_compass() &&
        effectiveMagCal != MagCal::EXTERNAL_YAW &&
        effectiveMagCal != MagCal::EXTERNAL_YAW_FALLBACK) {

        // because this type of reset event is not as time critical, require a continuous history of valid estimates
        if (!yawAlignComplete && EKFGSF_yaw_valid_count >= GSF_YAW_VALID_HISTORY_THRESHOLD) {
            yawAlignComplete = EKFGSF_resetMainFilterYaw();
        }

        if (imuSampleTime_ms - lastSynthYawTime_ms > 140) {
            if (fabsf(prevTnb[0][2]) < fabsf(prevTnb[1][2])) {
                // A 321 rotation order is best conditioned because the X axis is closer to horizontal than the Y axis
                yawAngDataDelayed.type = 2;
            } else {
                // A 312 rotation order is best conditioned because the Y axis is closer to horizontal than the X axis
                yawAngDataDelayed.type = 1;
            }

            float yawEKFGSF, yawVarianceEKFGSF;
            bool canUseEKFGSF = yawEstimator->getYawData(yawEKFGSF, yawVarianceEKFGSF) &&
                                is_positive(yawVarianceEKFGSF) && yawVarianceEKFGSF < sq(radians(GSF_YAW_ACCURACY_THRESHOLD_DEG));
            if (yawAlignComplete && canUseEKFGSF && !assume_zero_sideslip()) {
                // use the EKF-GSF yaw estimator output as this is more robust than the EKF can achieve without a yaw measurement
                // for non fixed wing platform types
                yawAngDataDelayed.yawAngErr = MAX(sqrtf(yawVarianceEKFGSF), 0.05f);
                yawAngDataDelayed.yawAng = yawEKFGSF;
                fuseEulerYaw(false, true);
            } else {
                // fuse the last dead-reckoned yaw when static to stop yaw drift and estimate yaw gyro bias estimate
                yawAngDataDelayed.yawAngErr = MAX(frontend->_yawNoise, 0.05f);
                if (!onGroundNotMoving) {
                    if (yawAngDataDelayed.type == 2) {
                        yawAngDataDelayed.yawAng = atan2f(prevTnb[0][1], prevTnb[0][0]);
                    } else if (yawAngDataDelayed.type == 1) {
                        yawAngDataDelayed.yawAng = atan2f(-prevTnb[0][1], prevTnb[1][1]);
                    }
                }
                if (onGroundNotMoving) {
                    // fuse last known good yaw angle before we stopped moving to allow yaw bias learning when on ground before flight
                    fuseEulerYaw(false, true);
                } else if (onGround || (sq(P[0][0])+sq(P[1][1])+sq(P[2][2])+sq(P[3][3]) > 0.01f)) {
                    // prevent uncontrolled yaw variance growth by fusing a zero innovation
                    // when not on ground allow more variance growth so yaw can be corrected
                    // by manoeuvring
                    fuseEulerYaw(true, true);
                }
            }
            magTestRatio.zero();
            yawTestRatio = 0.0f;
            lastSynthYawTime_ms = imuSampleTime_ms;
        }
        return;
    }

    // Handle case where we are using an external yaw sensor instead of a magnetomer
    if (effectiveMagCal == MagCal::EXTERNAL_YAW || effectiveMagCal == MagCal::EXTERNAL_YAW_FALLBACK) {
        bool have_fused_gps_yaw = false;
        if (storedYawAng.recall(yawAngDataDelayed,imuDataDelayed.time_ms)) {
            if (tiltAlignComplete && !yawAlignComplete) {
                alignYawAngle();
            } else if (tiltAlignComplete && yawAlignComplete) {
                fuseEulerYaw(false, true);
            }
            have_fused_gps_yaw = true;
            last_gps_yaw_fusion_ms = imuSampleTime_ms;
        } else if (tiltAlignComplete && !yawAlignComplete && (imuSampleTime_ms - lastSynthYawTime_ms > 140)) {
            yawAngDataDelayed.yawAngErr = MAX(frontend->_yawNoise, 0.05f);
            // update the yaw angle using the last estimate which will be used as a static yaw reference when movement stops
            if (fabsf(prevTnb[0][2]) < fabsf(prevTnb[1][2])) {
                // A 321 rotation order is best conditioned because the X axis is closer to horizontal than the Y axis
                if (!onGroundNotMoving) {
                    yawAngDataDelayed.yawAng = atan2f(prevTnb[0][1], prevTnb[0][0]);
                }
                yawAngDataDelayed.type = 2;
            } else {
                // A 312 rotation order is best conditioned because the Y axis is closer to horizontal than the X axis
                if (!onGroundNotMoving) {
                    yawAngDataDelayed.yawAng = atan2f(-prevTnb[0][1], prevTnb[1][1]);
                }
                yawAngDataDelayed.type = 1;
            }
            if (onGroundNotMoving) {
                // fuse last known good yaw angle before we stopped moving to allow yaw bias learning when on ground before flight
                fuseEulerYaw(false, true);
            } else {
                // prevent uncontrolled yaw variance growth by fusing a zero innovation
                fuseEulerYaw(true, true);
            }
            lastSynthYawTime_ms = imuSampleTime_ms;
        }
        if (effectiveMagCal == MagCal::EXTERNAL_YAW) {
            // no fallback
            return;
        }

        // get new mag data into delay buffer
        readMagData();

        if (have_fused_gps_yaw) {
            if (gps_yaw_mag_fallback_active) {
                gps_yaw_mag_fallback_active = false;
                gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u yaw external",(unsigned)imu_index);
            }
            // update mag bias from GPS yaw
            gps_yaw_mag_fallback_ok = learnMagBiasFromGPS();
            return;
        }

        // we don't have GPS yaw data and are configured for
        // fallback. If we've only just lost GPS yaw
        if (imuSampleTime_ms - last_gps_yaw_fusion_ms < 10000) {
            // don't fallback to magnetometer fusion for 10s
            return;
        }
        if (!gps_yaw_mag_fallback_ok) {
            // mag was not consistent enough with GPS to use it as
            // fallback
            return;
        }
        if (!inFlight) {
            // don't fall back if not flying
            return;
        }
        if (!gps_yaw_mag_fallback_active) {
            gps_yaw_mag_fallback_active = true;
            gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u yaw fallback active",(unsigned)imu_index);
        }
        // fall through to magnetometer fusion
    }

    if (effectiveMagCal != MagCal::EXTERNAL_YAW_FALLBACK) {
        // check for and read new magnetometer measurements. We don't
        // real for EXTERNAL_YAW_FALLBACK as it has already been read
        // above
        readMagData();
    }

    // If we are using the compass and the magnetometer has been unhealthy for too long we declare a timeout
    if (magHealth) {
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > frontend->magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

    // check for availability of magnetometer or other yaw data to fuse
    magDataToFuse = storedMag.recall(magDataDelayed,imuDataDelayed.time_ms);

    // Control reset of yaw and magnetic field states if we are using compass data
    if (magDataToFuse) {
        controlMagYawReset();
    }

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    bool dataReady = (magDataToFuse && statesInitialised && use_compass() && yawAlignComplete);
    if (dataReady) {
        // use the simple method of declination to maintain heading if we cannot use the magnetic field states
        if(inhibitMagStates || magStateResetRequest || !magStateInitComplete) {
            fuseEulerYaw(false, false);

            // zero the test ratio output from the inactive 3-axis magnetometer fusion
            magTestRatio.zero();

        } else {
            // if we are not doing aiding with earth relative observations (eg GPS) then the declination is
            // maintained by fusing declination as a synthesised observation
            // We also fuse declination if we are using the WMM tables
            if (PV_AidingMode != AID_ABSOLUTE ||
                (frontend->_mag_ef_limit > 0 && have_table_earth_field)) {
                FuseDeclination(0.34f);
            }
            // fuse the three magnetometer componenents using sequential fusion for each axis
            hal.util->perf_begin(_perf_test[0]);
            FuseMagnetometer();
            hal.util->perf_end(_perf_test[0]);
            // zero the test ratio output from the inactive simple magnetometer yaw fusion
            yawTestRatio = 0.0f;
        }
    }

    // If the final yaw reset has been performed and the state variances are sufficiently low
    // record that the earth field has been learned.
    if (!magFieldLearned && finalInflightMagInit) {
        magFieldLearned = (P[16][16] < sq(0.01f)) && (P[17][17] < sq(0.01f)) && (P[18][18] < sq(0.01f));
    }

    // record the last learned field variances
    if (magFieldLearned && !inhibitMagStates) {
        earthMagFieldVar.x = P[16][16];
        earthMagFieldVar.y = P[17][17];
        earthMagFieldVar.z = P[18][18];
        bodyMagFieldVar.x = P[19][19];
        bodyMagFieldVar.y = P[20][20];
        bodyMagFieldVar.z = P[21][21];
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseMagnetometer);
}

/*
 * Fuse magnetometer measurements using explicit algebraic equations auto-generated from
 * libraries/AP_NavEKF3/python/main.py with output recorded in libraries/AP_NavEKF3/python/3Dmag_generated.cpp
*/
void NavEKF3_core::FuseMagnetometer()
{
    // declarations
    ftype &q0 = mag_state.q0;
    ftype &q1 = mag_state.q1;
    ftype &q2 = mag_state.q2;
    ftype &q3 = mag_state.q3;
    ftype &magN = mag_state.magN;
    ftype &magE = mag_state.magE;
    ftype &magD = mag_state.magD;
    ftype &magXbias = mag_state.magXbias;
    ftype &magYbias = mag_state.magYbias;
    ftype &magZbias = mag_state.magZbias;
    Matrix3f &DCM = mag_state.DCM;
    Vector3f &MagPred = mag_state.MagPred;
    ftype &R_MAG = mag_state.R_MAG;
    Vector24 H_MAG;

    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    // calculate observation jacobians and Kalman gains

    // copy required states to local variable names
    q0       = stateStruct.quat[0];
    q1       = stateStruct.quat[1];
    q2       = stateStruct.quat[2];
    q3       = stateStruct.quat[3];
    magN     = stateStruct.earth_magfield[0];
    magE     = stateStruct.earth_magfield[1];
    magD     = stateStruct.earth_magfield[2];
    magXbias = stateStruct.body_magfield[0];
    magYbias = stateStruct.body_magfield[1];
    magZbias = stateStruct.body_magfield[2];

    // rotate predicted earth components into body axes and calculate
    // predicted measurements
    DCM[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    DCM[0][1] = 2.0f*(q1*q2 + q0*q3);
    DCM[0][2] = 2.0f*(q1*q3-q0*q2);
    DCM[1][0] = 2.0f*(q1*q2 - q0*q3);
    DCM[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    DCM[1][2] = 2.0f*(q2*q3 + q0*q1);
    DCM[2][0] = 2.0f*(q1*q3 + q0*q2);
    DCM[2][1] = 2.0f*(q2*q3 - q0*q1);
    DCM[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    MagPred[0] = DCM[0][0]*magN + DCM[0][1]*magE  + DCM[0][2]*magD + magXbias;
    MagPred[1] = DCM[1][0]*magN + DCM[1][1]*magE  + DCM[1][2]*magD + magYbias;
    MagPred[2] = DCM[2][0]*magN + DCM[2][1]*magE  + DCM[2][2]*magD + magZbias;

    // calculate the measurement innovation for each axis
    for (uint8_t i = 0; i<=2; i++) {
        innovMag[i] = MagPred[i] - magDataDelayed.mag[i];
    }

    // scale magnetometer observation error with total angular rate to allow for timing errors
    R_MAG = sq(constrain_float(frontend->_magNoise, 0.01f, 0.5f)) + sq(frontend->magVarRateScale*imuDataDelayed.delAng.length() / imuDataDelayed.delAngDT);

    // Calculate the innovation variance for each axis
    // X axis
    float SX0 = 2*q3;
    float SX1 = 2*q2;
    float SX2 = SX0*q0 + SX1*q1;
    float SX3 = SX0*q1 - SX1*q0;
    float SX4 = 2*magN;
    float SX5 = SX0*magE - SX1*magD + SX4*q0;
    float SX6 = SX0*magD + SX1*magE + SX4*q1;
    float SX7 = 2*magD;
    float SX8 = 2*magE;
    float SX9 = -SX1*magN - SX7*q0 + SX8*q1;
    float SX10 = -SX0*magN + SX7*q1 + SX8*q0;
    float SX11 = sq(q0) + sq(q1) - sq(q2) - sq(q3);

    varInnovMag[0] = P[0][19]*SX5 + P[16][19]*SX11 + P[17][19]*SX2 + P[18][19]*SX3 + P[19][19] + P[1][19]*SX6 + P[2][19]*SX9 + P[3][19]*SX10 + R_MAG + SX10*(P[0][3]*SX5 + P[1][3]*SX6 + P[2][3]*SX9 + P[3][16]*SX11 + P[3][17]*SX2 + P[3][18]*SX3 + P[3][19] + P[3][3]*SX10) + SX11*(P[0][16]*SX5 + P[16][16]*SX11 + P[16][17]*SX2 + P[16][18]*SX3 + P[16][19] + P[1][16]*SX6 + P[2][16]*SX9 + P[3][16]*SX10) + SX2*(P[0][17]*SX5 + P[16][17]*SX11 + P[17][17]*SX2 + P[17][18]*SX3 + P[17][19] + P[1][17]*SX6 + P[2][17]*SX9 + P[3][17]*SX10) + SX3*(P[0][18]*SX5 + P[16][18]*SX11 + P[17][18]*SX2 + P[18][18]*SX3 + P[18][19] + P[1][18]*SX6 + P[2][18]*SX9 + P[3][18]*SX10) + SX5*(P[0][0]*SX5 + P[0][16]*SX11 + P[0][17]*SX2 + P[0][18]*SX3 + P[0][19] + P[0][1]*SX6 + P[0][2]*SX9 + P[0][3]*SX10) + SX6*(P[0][1]*SX5 + P[1][16]*SX11 + P[1][17]*SX2 + P[1][18]*SX3 + P[1][19] + P[1][1]*SX6 + P[1][2]*SX9 + P[1][3]*SX10) + SX9*(P[0][2]*SX5 + P[1][2]*SX6 + P[2][16]*SX11 + P[2][17]*SX2 + P[2][18]*SX3 + P[2][19] + P[2][2]*SX9 + P[2][3]*SX10);

    if (varInnovMag[0] >= R_MAG) {
        faultStatus.bad_xmag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        faultStatus.bad_xmag = true;
        return;
    }

    // Y axis
    float SY0 = 2*q3;
    float SY1 = 2*q1;
    float SY2 = -SY0*q0 + SY1*q2;
    float SY3 = SY0*q2 + SY1*q0;
    float SY4 = 2*q0;
    float SY5 = -SY0*magN + SY1*magD + SY4*magE;
    float SY6 = 2*q2;
    float SY7 = -SY1*magE + SY4*magD + SY6*magN;
    float SY8 = SY0*magD + SY1*magN + SY6*magE;
    float SY9 = -SY0*magE - SY4*magN + SY6*magD;
    float SY10 = sq(q0) - sq(q1) + sq(q2) - sq(q3);

    varInnovMag[1] = P[0][20]*SY5 + P[16][20]*SY2 + P[17][20]*SY10 + P[18][20]*SY3 + P[1][20]*SY7 + P[20][20] + P[2][20]*SY8 + P[3][20]*SY9 + R_MAG + SY10*(P[0][17]*SY5 + P[16][17]*SY2 + P[17][17]*SY10 + P[17][18]*SY3 + P[17][20] + P[1][17]*SY7 + P[2][17]*SY8 + P[3][17]*SY9) + SY2*(P[0][16]*SY5 + P[16][16]*SY2 + P[16][17]*SY10 + P[16][18]*SY3 + P[16][20] + P[1][16]*SY7 + P[2][16]*SY8 + P[3][16]*SY9) + SY3*(P[0][18]*SY5 + P[16][18]*SY2 + P[17][18]*SY10 + P[18][18]*SY3 + P[18][20] + P[1][18]*SY7 + P[2][18]*SY8 + P[3][18]*SY9) + SY5*(P[0][0]*SY5 + P[0][16]*SY2 + P[0][17]*SY10 + P[0][18]*SY3 + P[0][1]*SY7 + P[0][20] + P[0][2]*SY8 + P[0][3]*SY9) + SY7*(P[0][1]*SY5 + P[1][16]*SY2 + P[1][17]*SY10 + P[1][18]*SY3 + P[1][1]*SY7 + P[1][20] + P[1][2]*SY8 + P[1][3]*SY9) + SY8*(P[0][2]*SY5 + P[1][2]*SY7 + P[2][16]*SY2 + P[2][17]*SY10 + P[2][18]*SY3 + P[2][20] + P[2][2]*SY8 + P[2][3]*SY9) + SY9*(P[0][3]*SY5 + P[1][3]*SY7 + P[2][3]*SY8 + P[3][16]*SY2 + P[3][17]*SY10 + P[3][18]*SY3 + P[3][20] + P[3][3]*SY9);

    if (varInnovMag[1] >= R_MAG) {
        faultStatus.bad_ymag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        faultStatus.bad_ymag = true;
        return;
    }

    // Z axis
    float SZ0 = 2*q0;
    float SZ1 = 2*q1;
    float SZ2 = SZ0*q2 + SZ1*q3;
    float SZ3 = 2*q2;
    float SZ4 = -SZ1*q0 + SZ3*q3;
    float SZ5 = SZ0*magD - SZ1*magE + SZ3*magN;
    float SZ6 = 2*q3;
    float SZ7 = -SZ0*magE - SZ1*magD + SZ6*magN;
    float SZ8 = SZ0*magN - SZ3*magD + SZ6*magE;
    float SZ9 = SZ1*magN + SZ3*magE + SZ6*magD;
    float SZ10 = sq(q0) - sq(q1) - sq(q2) + sq(q3);

    varInnovMag[2] = P[0][21]*SZ5 + P[16][21]*SZ2 + P[17][21]*SZ4 + P[18][21]*SZ10 + P[1][21]*SZ7 + P[21][21] + P[2][21]*SZ8 + P[3][21]*SZ9 + R_MAG + SZ10*(P[0][18]*SZ5 + P[16][18]*SZ2 + P[17][18]*SZ4 + P[18][18]*SZ10 + P[18][21] + P[1][18]*SZ7 + P[2][18]*SZ8 + P[3][18]*SZ9) + SZ2*(P[0][16]*SZ5 + P[16][16]*SZ2 + P[16][17]*SZ4 + P[16][18]*SZ10 + P[16][21] + P[1][16]*SZ7 + P[2][16]*SZ8 + P[3][16]*SZ9) + SZ4*(P[0][17]*SZ5 + P[16][17]*SZ2 + P[17][17]*SZ4 + P[17][18]*SZ10 + P[17][21] + P[1][17]*SZ7 + P[2][17]*SZ8 + P[3][17]*SZ9) + SZ5*(P[0][0]*SZ5 + P[0][16]*SZ2 + P[0][17]*SZ4 + P[0][18]*SZ10 + P[0][1]*SZ7 + P[0][21] + P[0][2]*SZ8 + P[0][3]*SZ9) + SZ7*(P[0][1]*SZ5 + P[1][16]*SZ2 + P[1][17]*SZ4 + P[1][18]*SZ10 + P[1][1]*SZ7 + P[1][21] + P[1][2]*SZ8 + P[1][3]*SZ9) + SZ8*(P[0][2]*SZ5 + P[1][2]*SZ7 + P[2][16]*SZ2 + P[2][17]*SZ4 + P[2][18]*SZ10 + P[2][21] + P[2][2]*SZ8 + P[2][3]*SZ9) + SZ9*(P[0][3]*SZ5 + P[1][3]*SZ7 + P[2][3]*SZ8 + P[3][16]*SZ2 + P[3][17]*SZ4 + P[3][18]*SZ10 + P[3][21] + P[3][3]*SZ9);

    if (varInnovMag[2] >= R_MAG) {
        faultStatus.bad_zmag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        faultStatus.bad_zmag = true;
        return;
    }

    // calculate the innovation test ratios
    for (uint8_t i = 0; i<=2; i++) {
        magTestRatio[i] = sq(innovMag[i]) / (sq(MAX(0.01f * (float)frontend->_magInnovGate, 1.0f)) * varInnovMag[i]);
    }

    // check the last values from all components and set magnetometer health accordingly
    magHealth = (magTestRatio[0] < 1.0f && magTestRatio[1] < 1.0f && magTestRatio[2] < 1.0f);

    // if the magnetometer is unhealthy, do not proceed further
    if (!magHealth) {
        return;
    }

    for (uint8_t obsIndex = 0; obsIndex <= 2; obsIndex++) {

        if (obsIndex == 0) { // Fuse X axis

            // calculate observation jacobians
            float HKX0 = 2*magD;
            float HKX1 = 2*magE;
            float HKX2 = 2*magN;
            float HKX3 = -HKX0*q2 + HKX1*q3 + HKX2*q0;
            float HKX4 = HKX0*q3 + HKX1*q2 + HKX2*q1;
            float HKX5 = -HKX0*q0 + HKX1*q1 - HKX2*q2;
            float HKX6 = HKX0*q1 + HKX1*q0 - HKX2*q3;
            float HKX7 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
            float HKX8 = 2*q0;
            float HKX9 = 2*q1;
            float HKX10 = HKX8*q3 + HKX9*q2;
            float HKX11 = -HKX8*q2 + HKX9*q3;
            float HKX12 = HKX10*P[0][17] + HKX11*P[0][18] + HKX3*P[0][0] + HKX4*P[0][1] + HKX5*P[0][2] + HKX6*P[0][3] + HKX7*P[0][16] + P[0][19];
            float HKX13 = HKX10*P[17][18] + HKX11*P[18][18] + HKX3*P[0][18] + HKX4*P[1][18] + HKX5*P[2][18] + HKX6*P[3][18] + HKX7*P[16][18] + P[18][19];
            float HKX14 = HKX10*P[17][17] + HKX11*P[17][18] + HKX3*P[0][17] + HKX4*P[1][17] + HKX5*P[2][17] + HKX6*P[3][17] + HKX7*P[16][17] + P[17][19];
            float HKX15 = HKX10*P[2][17] + HKX11*P[2][18] + HKX3*P[0][2] + HKX4*P[1][2] + HKX5*P[2][2] + HKX6*P[2][3] + HKX7*P[2][16] + P[2][19];
            float HKX16 = HKX10*P[3][17] + HKX11*P[3][18] + HKX3*P[0][3] + HKX4*P[1][3] + HKX5*P[2][3] + HKX6*P[3][3] + HKX7*P[3][16] + P[3][19];
            float HKX17 = HKX10*P[1][17] + HKX11*P[1][18] + HKX3*P[0][1] + HKX4*P[1][1] + HKX5*P[1][2] + HKX6*P[1][3] + HKX7*P[1][16] + P[1][19];
            float HKX18 = HKX10*P[16][17] + HKX11*P[16][18] + HKX3*P[0][16] + HKX4*P[1][16] + HKX5*P[2][16] + HKX6*P[3][16] + HKX7*P[16][16] + P[16][19];
            float HKX19 = HKX10*P[17][19] + HKX11*P[18][19] + HKX3*P[0][19] + HKX4*P[1][19] + HKX5*P[2][19] + HKX6*P[3][19] + HKX7*P[16][19] + P[19][19];
            float HKX20 = 1.0F/(HKX10*HKX14 + HKX11*HKX13 + HKX12*HKX3 + HKX15*HKX5 + HKX16*HKX6 + HKX17*HKX4 + HKX18*HKX7 + HKX19 + R_MAG);

            H_MAG[0] = HKX3;
            H_MAG[1] = HKX4;
            H_MAG[2] = HKX5;
            H_MAG[3] = HKX6;
            H_MAG[4] = 0;
            H_MAG[5] = 0;
            H_MAG[6] = 0;
            H_MAG[7] = 0;
            H_MAG[8] = 0;
            H_MAG[9] = 0;
            H_MAG[10] = 0;
            H_MAG[11] = 0;
            H_MAG[12] = 0;
            H_MAG[13] = 0;
            H_MAG[14] = 0;
            H_MAG[15] = 0;
            H_MAG[16] = HKX7;
            H_MAG[17] = HKX10;
            H_MAG[18] = HKX11;
            H_MAG[19] = 1;
            H_MAG[20] = 0;
            H_MAG[21] = 0;
            H_MAG[22] = 0;
            H_MAG[23] = 0;

            // calculate Kalman gains
            Kfusion[0] = HKX12*HKX20;
            Kfusion[1] = HKX17*HKX20;
            Kfusion[2] = HKX15*HKX20;
            Kfusion[3] = HKX16*HKX20;
            Kfusion[4] = HKX20*(HKX10*P[4][17] + HKX11*P[4][18] + HKX3*P[0][4] + HKX4*P[1][4] + HKX5*P[2][4] + HKX6*P[3][4] + HKX7*P[4][16] + P[4][19]);
            Kfusion[5] = HKX20*(HKX10*P[5][17] + HKX11*P[5][18] + HKX3*P[0][5] + HKX4*P[1][5] + HKX5*P[2][5] + HKX6*P[3][5] + HKX7*P[5][16] + P[5][19]);
            Kfusion[6] = HKX20*(HKX10*P[6][17] + HKX11*P[6][18] + HKX3*P[0][6] + HKX4*P[1][6] + HKX5*P[2][6] + HKX6*P[3][6] + HKX7*P[6][16] + P[6][19]);
            Kfusion[7] = HKX20*(HKX10*P[7][17] + HKX11*P[7][18] + HKX3*P[0][7] + HKX4*P[1][7] + HKX5*P[2][7] + HKX6*P[3][7] + HKX7*P[7][16] + P[7][19]);
            Kfusion[8] = HKX20*(HKX10*P[8][17] + HKX11*P[8][18] + HKX3*P[0][8] + HKX4*P[1][8] + HKX5*P[2][8] + HKX6*P[3][8] + HKX7*P[8][16] + P[8][19]);
            Kfusion[9] = HKX20*(HKX10*P[9][17] + HKX11*P[9][18] + HKX3*P[0][9] + HKX4*P[1][9] + HKX5*P[2][9] + HKX6*P[3][9] + HKX7*P[9][16] + P[9][19]);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = HKX20*(HKX10*P[10][17] + HKX11*P[10][18] + HKX3*P[0][10] + HKX4*P[1][10] + HKX5*P[2][10] + HKX6*P[3][10] + HKX7*P[10][16] + P[10][19]);
                Kfusion[11] = HKX20*(HKX10*P[11][17] + HKX11*P[11][18] + HKX3*P[0][11] + HKX4*P[1][11] + HKX5*P[2][11] + HKX6*P[3][11] + HKX7*P[11][16] + P[11][19]);
                Kfusion[12] = HKX20*(HKX10*P[12][17] + HKX11*P[12][18] + HKX3*P[0][12] + HKX4*P[1][12] + HKX5*P[2][12] + HKX6*P[3][12] + HKX7*P[12][16] + P[12][19]);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = HKX20*(HKX10*P[13][17] + HKX11*P[13][18] + HKX3*P[0][13] + HKX4*P[1][13] + HKX5*P[2][13] + HKX6*P[3][13] + HKX7*P[13][16] + P[13][19]);
                Kfusion[14] = HKX20*(HKX10*P[14][17] + HKX11*P[14][18] + HKX3*P[0][14] + HKX4*P[1][14] + HKX5*P[2][14] + HKX6*P[3][14] + HKX7*P[14][16] + P[14][19]);
                Kfusion[15] = HKX20*(HKX10*P[15][17] + HKX11*P[15][18] + HKX3*P[0][15] + HKX4*P[1][15] + HKX5*P[2][15] + HKX6*P[3][15] + HKX7*P[15][16] + P[15][19]);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = HKX18*HKX20;
                Kfusion[17] = HKX14*HKX20;
                Kfusion[18] = HKX13*HKX20;
                Kfusion[19] = HKX19*HKX20;
                Kfusion[20] = HKX20*(HKX10*P[17][20] + HKX11*P[18][20] + HKX3*P[0][20] + HKX4*P[1][20] + HKX5*P[2][20] + HKX6*P[3][20] + HKX7*P[16][20] + P[19][20]);
                Kfusion[21] = HKX20*(HKX10*P[17][21] + HKX11*P[18][21] + HKX3*P[0][21] + HKX4*P[1][21] + HKX5*P[2][21] + HKX6*P[3][21] + HKX7*P[16][21] + P[19][21]);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = HKX20*(HKX10*P[17][22] + HKX11*P[18][22] + HKX3*P[0][22] + HKX4*P[1][22] + HKX5*P[2][22] + HKX6*P[3][22] + HKX7*P[16][22] + P[19][22]);
                Kfusion[23] = HKX20*(HKX10*P[17][23] + HKX11*P[18][23] + HKX3*P[0][23] + HKX4*P[1][23] + HKX5*P[2][23] + HKX6*P[3][23] + HKX7*P[16][23] + P[19][23]);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
            magFuseRequired = true;

        } else if (obsIndex == 1) { // Fuse Y axis

            // calculate observation jacobians
            float HKY0 = 2*q1;
            float HKY1 = 2*q0;
            float HKY2 = 2*magN;
            float HKY3 = HKY0*magD + HKY1*magE - HKY2*q3;
            float HKY4 = -HKY0*magE + HKY1*magD + HKY2*q2;
            float HKY5 = 2*q3;
            float HKY6 = 2*q2;
            float HKY7 = HKY2*q1 + HKY5*magD + HKY6*magE;
            float HKY8 = -HKY2*q0 - HKY5*magE + HKY6*magD;
            float HKY9 = HKY0*q2 - HKY1*q3;
            float HKY10 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
            float HKY11 = HKY0*q0 + HKY5*q2;
            float HKY12 = HKY10*P[0][17] + HKY11*P[0][18] + HKY3*P[0][0] + HKY4*P[0][1] + HKY7*P[0][2] + HKY8*P[0][3] + HKY9*P[0][16] + P[0][20];
            float HKY13 = HKY10*P[16][17] + HKY11*P[16][18] + HKY3*P[0][16] + HKY4*P[1][16] + HKY7*P[2][16] + HKY8*P[3][16] + HKY9*P[16][16] + P[16][20];
            float HKY14 = HKY10*P[17][18] + HKY11*P[18][18] + HKY3*P[0][18] + HKY4*P[1][18] + HKY7*P[2][18] + HKY8*P[3][18] + HKY9*P[16][18] + P[18][20];
            float HKY15 = HKY10*P[1][17] + HKY11*P[1][18] + HKY3*P[0][1] + HKY4*P[1][1] + HKY7*P[1][2] + HKY8*P[1][3] + HKY9*P[1][16] + P[1][20];
            float HKY16 = HKY10*P[3][17] + HKY11*P[3][18] + HKY3*P[0][3] + HKY4*P[1][3] + HKY7*P[2][3] + HKY8*P[3][3] + HKY9*P[3][16] + P[3][20];
            float HKY17 = HKY10*P[2][17] + HKY11*P[2][18] + HKY3*P[0][2] + HKY4*P[1][2] + HKY7*P[2][2] + HKY8*P[2][3] + HKY9*P[2][16] + P[2][20];
            float HKY18 = HKY10*P[17][17] + HKY11*P[17][18] + HKY3*P[0][17] + HKY4*P[1][17] + HKY7*P[2][17] + HKY8*P[3][17] + HKY9*P[16][17] + P[17][20];
            float HKY19 = HKY10*P[17][20] + HKY11*P[18][20] + HKY3*P[0][20] + HKY4*P[1][20] + HKY7*P[2][20] + HKY8*P[3][20] + HKY9*P[16][20] + P[20][20];
            float HKY20 = 1.0F/(HKY10*HKY18 + HKY11*HKY14 + HKY12*HKY3 + HKY13*HKY9 + HKY15*HKY4 + HKY16*HKY8 + HKY17*HKY7 + HKY19 + R_MAG);

            H_MAG[0] = HKY3;
            H_MAG[1] = HKY4;
            H_MAG[2] = HKY7;
            H_MAG[3] = HKY8;
            H_MAG[4] = 0;
            H_MAG[5] = 0;
            H_MAG[6] = 0;
            H_MAG[7] = 0;
            H_MAG[8] = 0;
            H_MAG[9] = 0;
            H_MAG[10] = 0;
            H_MAG[11] = 0;
            H_MAG[12] = 0;
            H_MAG[13] = 0;
            H_MAG[14] = 0;
            H_MAG[15] = 0;
            H_MAG[16] = HKY9;
            H_MAG[17] = HKY10;
            H_MAG[18] = HKY11;
            H_MAG[19] = 0;
            H_MAG[20] = 1;
            H_MAG[21] = 0;
            H_MAG[22] = 0;
            H_MAG[23] = 0;

            // calculate Kalman gains
            Kfusion[0] = HKY12*HKY20;
            Kfusion[1] = HKY15*HKY20;
            Kfusion[2] = HKY17*HKY20;
            Kfusion[3] = HKY16*HKY20;
            Kfusion[4] = HKY20*(HKY10*P[4][17] + HKY11*P[4][18] + HKY3*P[0][4] + HKY4*P[1][4] + HKY7*P[2][4] + HKY8*P[3][4] + HKY9*P[4][16] + P[4][20]);
            Kfusion[5] = HKY20*(HKY10*P[5][17] + HKY11*P[5][18] + HKY3*P[0][5] + HKY4*P[1][5] + HKY7*P[2][5] + HKY8*P[3][5] + HKY9*P[5][16] + P[5][20]);
            Kfusion[6] = HKY20*(HKY10*P[6][17] + HKY11*P[6][18] + HKY3*P[0][6] + HKY4*P[1][6] + HKY7*P[2][6] + HKY8*P[3][6] + HKY9*P[6][16] + P[6][20]);
            Kfusion[7] = HKY20*(HKY10*P[7][17] + HKY11*P[7][18] + HKY3*P[0][7] + HKY4*P[1][7] + HKY7*P[2][7] + HKY8*P[3][7] + HKY9*P[7][16] + P[7][20]);
            Kfusion[8] = HKY20*(HKY10*P[8][17] + HKY11*P[8][18] + HKY3*P[0][8] + HKY4*P[1][8] + HKY7*P[2][8] + HKY8*P[3][8] + HKY9*P[8][16] + P[8][20]);
            Kfusion[9] = HKY20*(HKY10*P[9][17] + HKY11*P[9][18] + HKY3*P[0][9] + HKY4*P[1][9] + HKY7*P[2][9] + HKY8*P[3][9] + HKY9*P[9][16] + P[9][20]);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = HKY20*(HKY10*P[10][17] + HKY11*P[10][18] + HKY3*P[0][10] + HKY4*P[1][10] + HKY7*P[2][10] + HKY8*P[3][10] + HKY9*P[10][16] + P[10][20]);
                Kfusion[11] = HKY20*(HKY10*P[11][17] + HKY11*P[11][18] + HKY3*P[0][11] + HKY4*P[1][11] + HKY7*P[2][11] + HKY8*P[3][11] + HKY9*P[11][16] + P[11][20]);
                Kfusion[12] = HKY20*(HKY10*P[12][17] + HKY11*P[12][18] + HKY3*P[0][12] + HKY4*P[1][12] + HKY7*P[2][12] + HKY8*P[3][12] + HKY9*P[12][16] + P[12][20]);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = HKY20*(HKY10*P[13][17] + HKY11*P[13][18] + HKY3*P[0][13] + HKY4*P[1][13] + HKY7*P[2][13] + HKY8*P[3][13] + HKY9*P[13][16] + P[13][20]);
                Kfusion[14] = HKY20*(HKY10*P[14][17] + HKY11*P[14][18] + HKY3*P[0][14] + HKY4*P[1][14] + HKY7*P[2][14] + HKY8*P[3][14] + HKY9*P[14][16] + P[14][20]);
                Kfusion[15] = HKY20*(HKY10*P[15][17] + HKY11*P[15][18] + HKY3*P[0][15] + HKY4*P[1][15] + HKY7*P[2][15] + HKY8*P[3][15] + HKY9*P[15][16] + P[15][20]);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = HKY13*HKY20;
                Kfusion[17] = HKY18*HKY20;
                Kfusion[18] = HKY14*HKY20;
                Kfusion[19] = HKY20*(HKY10*P[17][19] + HKY11*P[18][19] + HKY3*P[0][19] + HKY4*P[1][19] + HKY7*P[2][19] + HKY8*P[3][19] + HKY9*P[16][19] + P[19][20]);
                Kfusion[20] = HKY19*HKY20;
                Kfusion[21] = HKY20*(HKY10*P[17][21] + HKY11*P[18][21] + HKY3*P[0][21] + HKY4*P[1][21] + HKY7*P[2][21] + HKY8*P[3][21] + HKY9*P[16][21] + P[20][21]);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = HKY20*(HKY10*P[17][22] + HKY11*P[18][22] + HKY3*P[0][22] + HKY4*P[1][22] + HKY7*P[2][22] + HKY8*P[3][22] + HKY9*P[16][22] + P[20][22]);
                Kfusion[23] = HKY20*(HKY10*P[17][23] + HKY11*P[18][23] + HKY3*P[0][23] + HKY4*P[1][23] + HKY7*P[2][23] + HKY8*P[3][23] + HKY9*P[16][23] + P[20][23]);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
            magFuseRequired = true;
        }
        else if (obsIndex == 2) // Fuse Z axis
        {
            // calculate observation jacobians
            float HKZ0 = 2*q0;
            float HKZ1 = 2*q1;
            float HKZ2 = 2*magN;
            float HKZ3 = HKZ0*magD - HKZ1*magE + HKZ2*q2;
            float HKZ4 = -HKZ0*magE - HKZ1*magD + HKZ2*q3;
            float HKZ5 = 2*magD;
            float HKZ6 = 2*magE;
            float HKZ7 = HKZ0*magN - HKZ5*q2 + HKZ6*q3;
            float HKZ8 = HKZ1*magN + HKZ5*q3 + HKZ6*q2;
            float HKZ9 = HKZ0*q2 + HKZ1*q3;
            float HKZ10 = -HKZ1*q0 + 2*q2*q3;
            float HKZ11 = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            float HKZ12 = HKZ10*P[0][17] + HKZ11*P[0][18] + HKZ3*P[0][0] + HKZ4*P[0][1] + HKZ7*P[0][2] + HKZ8*P[0][3] + HKZ9*P[0][16] + P[0][21];
            float HKZ13 = HKZ10*P[17][17] + HKZ11*P[17][18] + HKZ3*P[0][17] + HKZ4*P[1][17] + HKZ7*P[2][17] + HKZ8*P[3][17] + HKZ9*P[16][17] + P[17][21];
            float HKZ14 = HKZ10*P[16][17] + HKZ11*P[16][18] + HKZ3*P[0][16] + HKZ4*P[1][16] + HKZ7*P[2][16] + HKZ8*P[3][16] + HKZ9*P[16][16] + P[16][21];
            float HKZ15 = HKZ10*P[1][17] + HKZ11*P[1][18] + HKZ3*P[0][1] + HKZ4*P[1][1] + HKZ7*P[1][2] + HKZ8*P[1][3] + HKZ9*P[1][16] + P[1][21];
            float HKZ16 = HKZ10*P[2][17] + HKZ11*P[2][18] + HKZ3*P[0][2] + HKZ4*P[1][2] + HKZ7*P[2][2] + HKZ8*P[2][3] + HKZ9*P[2][16] + P[2][21];
            float HKZ17 = HKZ10*P[3][17] + HKZ11*P[3][18] + HKZ3*P[0][3] + HKZ4*P[1][3] + HKZ7*P[2][3] + HKZ8*P[3][3] + HKZ9*P[3][16] + P[3][21];
            float HKZ18 = HKZ10*P[17][18] + HKZ11*P[18][18] + HKZ3*P[0][18] + HKZ4*P[1][18] + HKZ7*P[2][18] + HKZ8*P[3][18] + HKZ9*P[16][18] + P[18][21];
            float HKZ19 = HKZ10*P[17][21] + HKZ11*P[18][21] + HKZ3*P[0][21] + HKZ4*P[1][21] + HKZ7*P[2][21] + HKZ8*P[3][21] + HKZ9*P[16][21] + P[21][21];
            float HKZ20 = 1.0F/(HKZ10*HKZ13 + HKZ11*HKZ18 + HKZ12*HKZ3 + HKZ14*HKZ9 + HKZ15*HKZ4 + HKZ16*HKZ7 + HKZ17*HKZ8 + HKZ19 + R_MAG);

            H_MAG[0] = HKZ3;
            H_MAG[1] = HKZ4;
            H_MAG[2] = HKZ7;
            H_MAG[3] = HKZ8;
            H_MAG[4] = 0;
            H_MAG[5] = 0;
            H_MAG[6] = 0;
            H_MAG[7] = 0;
            H_MAG[8] = 0;
            H_MAG[9] = 0;
            H_MAG[10] = 0;
            H_MAG[11] = 0;
            H_MAG[12] = 0;
            H_MAG[13] = 0;
            H_MAG[14] = 0;
            H_MAG[15] = 0;
            H_MAG[16] = HKZ9;
            H_MAG[17] = HKZ10;
            H_MAG[18] = HKZ11;
            H_MAG[19] = 0;
            H_MAG[20] = 0;
            H_MAG[21] = 1;
            H_MAG[22] = 0;
            H_MAG[23] = 0;

            // calculate Kalman gain
            Kfusion[0] = HKZ12*HKZ20;
            Kfusion[1] = HKZ15*HKZ20;
            Kfusion[2] = HKZ16*HKZ20;
            Kfusion[3] = HKZ17*HKZ20;
            Kfusion[4] = HKZ20*(HKZ10*P[4][17] + HKZ11*P[4][18] + HKZ3*P[0][4] + HKZ4*P[1][4] + HKZ7*P[2][4] + HKZ8*P[3][4] + HKZ9*P[4][16] + P[4][21]);
            Kfusion[5] = HKZ20*(HKZ10*P[5][17] + HKZ11*P[5][18] + HKZ3*P[0][5] + HKZ4*P[1][5] + HKZ7*P[2][5] + HKZ8*P[3][5] + HKZ9*P[5][16] + P[5][21]);
            Kfusion[6] = HKZ20*(HKZ10*P[6][17] + HKZ11*P[6][18] + HKZ3*P[0][6] + HKZ4*P[1][6] + HKZ7*P[2][6] + HKZ8*P[3][6] + HKZ9*P[6][16] + P[6][21]);
            Kfusion[7] = HKZ20*(HKZ10*P[7][17] + HKZ11*P[7][18] + HKZ3*P[0][7] + HKZ4*P[1][7] + HKZ7*P[2][7] + HKZ8*P[3][7] + HKZ9*P[7][16] + P[7][21]);
            Kfusion[8] = HKZ20*(HKZ10*P[8][17] + HKZ11*P[8][18] + HKZ3*P[0][8] + HKZ4*P[1][8] + HKZ7*P[2][8] + HKZ8*P[3][8] + HKZ9*P[8][16] + P[8][21]);
            Kfusion[9] = HKZ20*(HKZ10*P[9][17] + HKZ11*P[9][18] + HKZ3*P[0][9] + HKZ4*P[1][9] + HKZ7*P[2][9] + HKZ8*P[3][9] + HKZ9*P[9][16] + P[9][21]);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = HKZ20*(HKZ10*P[10][17] + HKZ11*P[10][18] + HKZ3*P[0][10] + HKZ4*P[1][10] + HKZ7*P[2][10] + HKZ8*P[3][10] + HKZ9*P[10][16] + P[10][21]);
                Kfusion[11] = HKZ20*(HKZ10*P[11][17] + HKZ11*P[11][18] + HKZ3*P[0][11] + HKZ4*P[1][11] + HKZ7*P[2][11] + HKZ8*P[3][11] + HKZ9*P[11][16] + P[11][21]);
                Kfusion[12] = HKZ20*(HKZ10*P[12][17] + HKZ11*P[12][18] + HKZ3*P[0][12] + HKZ4*P[1][12] + HKZ7*P[2][12] + HKZ8*P[3][12] + HKZ9*P[12][16] + P[12][21]);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = HKZ20*(HKZ10*P[13][17] + HKZ11*P[13][18] + HKZ3*P[0][13] + HKZ4*P[1][13] + HKZ7*P[2][13] + HKZ8*P[3][13] + HKZ9*P[13][16] + P[13][21]);
                Kfusion[14] = HKZ20*(HKZ10*P[14][17] + HKZ11*P[14][18] + HKZ3*P[0][14] + HKZ4*P[1][14] + HKZ7*P[2][14] + HKZ8*P[3][14] + HKZ9*P[14][16] + P[14][21]);
                Kfusion[15] = HKZ20*(HKZ10*P[15][17] + HKZ11*P[15][18] + HKZ3*P[0][15] + HKZ4*P[1][15] + HKZ7*P[2][15] + HKZ8*P[3][15] + HKZ9*P[15][16] + P[15][21]);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = HKZ14*HKZ20;
                Kfusion[17] = HKZ13*HKZ20;
                Kfusion[18] = HKZ18*HKZ20;
                Kfusion[19] = HKZ20*(HKZ10*P[17][19] + HKZ11*P[18][19] + HKZ3*P[0][19] + HKZ4*P[1][19] + HKZ7*P[2][19] + HKZ8*P[3][19] + HKZ9*P[16][19] + P[19][21]);
                Kfusion[20] = HKZ20*(HKZ10*P[17][20] + HKZ11*P[18][20] + HKZ3*P[0][20] + HKZ4*P[1][20] + HKZ7*P[2][20] + HKZ8*P[3][20] + HKZ9*P[16][20] + P[20][21]);
                Kfusion[21] = HKZ19*HKZ20;
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = HKZ20*(HKZ10*P[17][22] + HKZ11*P[18][22] + HKZ3*P[0][22] + HKZ4*P[1][22] + HKZ7*P[2][22] + HKZ8*P[3][22] + HKZ9*P[16][22] + P[21][22]);
                Kfusion[23] = HKZ20*(HKZ10*P[17][23] + HKZ11*P[18][23] + HKZ3*P[0][23] + HKZ4*P[1][23] + HKZ7*P[2][23] + HKZ8*P[3][23] + HKZ9*P[16][23] + P[21][23]);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
        }
        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=3; j++) {
                KH[i][j] = Kfusion[i] * H_MAG[j];
            }
            for (unsigned j = 4; j<=15; j++) {
                KH[i][j] = 0.0f;
            }
            for (unsigned j = 16; j<=21; j++) {
                KH[i][j] = Kfusion[i] * H_MAG[j];
            }
            for (unsigned j = 22; j<=23; j++) {
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
                res += KH[i][16] * P[16][j];
                res += KH[i][17] * P[17][j];
                res += KH[i][18] * P[18][j];
                res += KH[i][19] * P[19][j];
                res += KH[i][20] * P[20][j];
                res += KH[i][21] * P[21][j];
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
                statesArray[j] = statesArray[j] - Kfusion[j] * innovMag[obsIndex];
            }

            // add table constraint here for faster convergence
            if (have_table_earth_field && frontend->_mag_ef_limit > 0) {
                MagTableConstrain();
            }

            stateStruct.quat.normalize();

        } else {
            // record bad axis
            if (obsIndex == 0) {
                faultStatus.bad_xmag = true;
            } else if (obsIndex == 1) {
                faultStatus.bad_ymag = true;
            } else if (obsIndex == 2) {
                faultStatus.bad_zmag = true;
            }
            CovarianceInit();
            return;
        }
    }
}


/*
 * Fuse yaw measurements using explicit algebraic equations auto-generated from
 * libraries/AP_NavEKF3/python/main.py with output recorded in libraries/AP_NavEKF3/python/yaw_generated.cpp
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computationally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magnetometer failures.
 * It is not suitable for operation where the horizontal magnetic field strength is weak (within 30 degrees latitude of the magnetic poles)
 *
 * The following booleans are passed to the function to control the fusion process:
 *
 * usePredictedYaw -  Set this to true if no valid yaw measurement will be available for an extended periods.
 *                    This uses an innovation set to zero which prevents uncontrolled quaternion covariance
 *                    growth or if available, a yaw estimate from the Gaussian Sum Filter.
 * UseExternalYawSensor - Set this to true if yaw data from an external yaw sensor (GPS or external nav) is being used instead of the magnetometer.
*/
void NavEKF3_core::fuseEulerYaw(bool usePredictedYaw, bool useExternalYawSensor)
{
    const float &q0 = stateStruct.quat[0];
    const float &q1 = stateStruct.quat[1];
    const float &q2 = stateStruct.quat[2];
    const float &q3 = stateStruct.quat[3];

    // external yaw available check
    bool canUseGsfYaw = false;
    float gsfYaw = 0.0f;
    float gsfYawVariance = 0.0f;
    if (usePredictedYaw && yawEstimator != nullptr) {
        canUseGsfYaw = yawEstimator->getYawData(gsfYaw, gsfYawVariance)
                        && is_positive(gsfYawVariance)
                        && gsfYawVariance < sq(radians(GSF_YAW_ACCURACY_THRESHOLD_DEG));
    }

    // yaw measurement error variance (rad^2)
    float R_YAW;
    if (canUseGsfYaw) {
        R_YAW = gsfYawVariance;
    } else if (!useExternalYawSensor) {
        R_YAW = sq(frontend->_yawNoise);
    } else {
        R_YAW = sq(yawAngDataDelayed.yawAngErr);
    }

    // determine if a 321 or 312 Euler sequence is best
    bool useEuler321 = true;
    if (useExternalYawSensor) {
        // If using an external sensor, the definition of yaw is specified through the sensor interface
        if (yawAngDataDelayed.type == 2) {
            useEuler321 = true;
        } else if (yawAngDataDelayed.type == 1) {
            useEuler321 = false;
        } else {
            // invalid selection
            return;
        }
    } else {
        // if using the magnetometer, it is determined automatically
        useEuler321 = (fabsf(prevTnb[0][2]) < fabsf(prevTnb[1][2]));
    }

    // calculate observation jacobian, predicted yaw and zero yaw body to earth rotation matrix
    float yawAngPredicted;
    float H_YAW[4];
    Matrix3f Tbn_zeroYaw;

    if (useEuler321) {
        // calculate 321 yaw observation matrix - option A or B to avoid singularity in derivation at +-90 degrees yaw
        bool canUseA = false;
        const float SA0 = 2*q3;
        const float SA1 = 2*q2;
        const float SA2 = SA0*q0 + SA1*q1;
        const float SA3 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        float SA4, SA5_inv;
        if (is_positive(sq(SA3))) {
            SA4 = 1.0F/sq(SA3);
            SA5_inv = sq(SA2)*SA4 + 1;
            canUseA = is_positive(fabsf(SA5_inv));
        }

        bool canUseB = false;
        const float SB0 = 2*q0;
        const float SB1 = 2*q1;
        const float SB2 = SB0*q3 + SB1*q2;
        const float SB4 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        float SB3, SB5_inv;
        if (is_positive(sq(SB2))) {
            SB3 = 1.0F/sq(SB2);
            SB5_inv = SB3*sq(SB4) + 1;
            canUseB = is_positive(fabsf(SB5_inv));
        }

        if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
            const float SA5 = 1.0F/SA5_inv;
            const float SA6 = 1.0F/SA3;
            const float SA7 = SA2*SA4;
            const float SA8 = 2*SA7;
            const float SA9 = 2*SA6;

            H_YAW[0] = SA5*(SA0*SA6 - SA8*q0);
            H_YAW[1] = SA5*(SA1*SA6 - SA8*q1);
            H_YAW[2] = SA5*(SA1*SA7 + SA9*q1);
            H_YAW[3] = SA5*(SA0*SA7 + SA9*q0);
        } else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
            const float SB5 = 1.0F/SB5_inv;
            const float SB6 = 1.0F/SB2;
            const float SB7 = SB3*SB4;
            const float SB8 = 2*SB7;
            const float SB9 = 2*SB6;

            H_YAW[0] = -SB5*(SB0*SB6 - SB8*q3);
            H_YAW[1] = -SB5*(SB1*SB6 - SB8*q2);
            H_YAW[2] = -SB5*(-SB1*SB7 - SB9*q2);
            H_YAW[3] = -SB5*(-SB0*SB7 - SB9*q3);
        } else {
            return;
        }

        // Get the 321 euler angles
        Vector3f euler321;
        stateStruct.quat.to_euler(euler321.x, euler321.y, euler321.z);
        yawAngPredicted = euler321.z;

        // set the yaw to zero and calculate the zero yaw rotation from body to earth frame
        Tbn_zeroYaw.from_euler(euler321.x, euler321.y, 0.0f);

    } else {
        // calculate 312 yaw observation matrix - option A or B to avoid singularity in derivation at +-90 degrees yaw
        bool canUseA = false;
        const float SA0 = 2*q3;
        const float SA1 = 2*q2;
        const float SA2 = SA0*q0 - SA1*q1;
        const float SA3 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
        float SA4, SA5_inv;
        if (is_positive(sq(SA3))) {
            SA4 = 1.0F/sq(SA3);
            SA5_inv = sq(SA2)*SA4 + 1;
            canUseA = is_positive(fabsf(SA5_inv));
        }

        bool canUseB = false;
        const float SB0 = 2*q0;
        const float SB1 = 2*q1;
        const float SB2 = -SB0*q3 + SB1*q2;
        const float SB4 = -sq(q0) + sq(q1) - sq(q2) + sq(q3);
        float SB3, SB5_inv;
        if (is_positive(sq(SB2))) {
            SB3 = 1.0F/sq(SB2);
            SB5_inv = SB3*sq(SB4) + 1;
            canUseB = is_positive(fabsf(SB5_inv));
        }

        if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
            const float SA5 = 1.0F/SA5_inv;
            const float SA6 = 1.0F/SA3;
            const float SA7 = SA2*SA4;
            const float SA8 = 2*SA7;
            const float SA9 = 2*SA6;

            H_YAW[0] = SA5*(SA0*SA6 - SA8*q0);
            H_YAW[1] = SA5*(-SA1*SA6 + SA8*q1);
            H_YAW[2] = SA5*(-SA1*SA7 - SA9*q1);
            H_YAW[3] = SA5*(SA0*SA7 + SA9*q0);
        } else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
            const float SB5 = 1.0F/SB5_inv;
            const float SB6 = 1.0F/SB2;
            const float SB7 = SB3*SB4;
            const float SB8 = 2*SB7;
            const float SB9 = 2*SB6;

            H_YAW[0] = -SB5*(-SB0*SB6 + SB8*q3);
            H_YAW[1] = -SB5*(SB1*SB6 - SB8*q2);
            H_YAW[2] = -SB5*(-SB1*SB7 - SB9*q2);
            H_YAW[3] = -SB5*(SB0*SB7 + SB9*q3);
        } else {
            return;
        }

        // Get the 312 Tait Bryan rotation angles
        Vector3f euler312 = stateStruct.quat.to_vector312();
        yawAngPredicted = euler312.z;

        // set the yaw to zero and calculate the zero yaw rotation from body to earth frame
        Tbn_zeroYaw.from_euler312(euler312.x, euler312.y, 0.0f);
    }

    // Calculate the innovation
    float innovation;
    if (!usePredictedYaw) {
        if (!useExternalYawSensor) {
            // Use the difference between the horizontal projection and declination to give the measured yaw
            // rotate measured mag components into earth frame
            Vector3f magMeasNED = Tbn_zeroYaw*magDataDelayed.mag;
            float yawAngMeasured = wrap_PI(-atan2f(magMeasNED.y, magMeasNED.x) + MagDeclination());
            innovation = wrap_PI(yawAngPredicted - yawAngMeasured);
        } else {
            // use the external yaw sensor data
            innovation = wrap_PI(yawAngPredicted - yawAngDataDelayed.yawAng);
        }
    } else if (canUseGsfYaw) {
        // The GSF yaw esitimator can provide a better estimate than the main nav filter can when no yaw
        // sensor is available
        innovation = wrap_PI(yawAngPredicted - gsfYaw);
    } else {
        // setting the innovation to zero enables quaternion covariance growth to be constrained when there
        // is no method of observing yaw
        innovation = 0.0f;
    }

    // Copy raw value to output variable used for data logging
    innovYaw = innovation;

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 4 elements in H are non zero
    float PH[4];
    float varInnov = R_YAW;
    for (uint8_t rowIndex=0; rowIndex<=3; rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=3; colIndex++) {
            PH[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        varInnov += H_YAW[rowIndex]*PH[rowIndex];
    }
    float varInnovInv;
    if (varInnov >= R_YAW) {
        varInnovInv = 1.0f / varInnov;
        // output numerical health status
        faultStatus.bad_yaw = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        // output numerical health status
        faultStatus.bad_yaw = true;
        return;
    }

    // calculate Kalman gain
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        Kfusion[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=3; colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }

    // calculate the innovation test ratio
    yawTestRatio = sq(innovation) / (sq(MAX(0.01f * (float)frontend->_yawInnovGate, 1.0f)) * varInnov);

    // Declare the magnetometer unhealthy if the innovation test fails
    if (yawTestRatio > 1.0f) {
        magHealth = false;
        // On the ground a large innovation could be due to large initial gyro bias or magnetic interference from nearby objects
        // If we are flying, then it is more likely due to a magnetometer fault and we should not fuse the data
        if (inFlight) {
            return;
        }
    } else {
        magHealth = true;
    }

    // limit the innovation so that initial corrections are not too large
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    // calculate K*H*P
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= 3; column++) {
            KH[row][column] = Kfusion[row] * H_YAW[column];
        }
    }
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= stateIndexLim; column++) {
            float tmp = KH[row][0] * P[0][column];
            tmp += KH[row][1] * P[1][column];
            tmp += KH[row][2] * P[2][column];
            tmp += KH[row][3] * P[3][column];
            KHP[row][column] = tmp;
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
        for (uint8_t i=0; i<=stateIndexLim; i++) {
            statesArray[i] -= Kfusion[i] * innovation;
        }
        stateStruct.quat.normalize();

        // record fusion numerical health status
        faultStatus.bad_yaw = false;

    } else {
        // record fusion numerical health status
        faultStatus.bad_yaw = true;
    }
}

/*
 * Fuse declination angle using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * This is used to prevent the declination of the EKF earth field states from drifting during operation without GPS
 * or some other absolute position or velocity reference
*/
void NavEKF3_core::FuseDeclination(float declErr)
{
    // declination error variance (rad^2)
    const float R_DECL = sq(declErr);

    // copy required states to local variables
    float magN = stateStruct.earth_magfield.x;
    float magE = stateStruct.earth_magfield.y;

    // prevent bad earth field states from causing numerical errors or exceptions
    if (magN < 1e-3f) {
        return;
    }

    // Calculate observation Jacobian and Kalman gains
    // Calculate intermediate variables
    float t2 = magE*magE;
    float t3 = magN*magN;
    float t4 = t2+t3;
    // if the horizontal magnetic field is too small, this calculation will be badly conditioned
    if (t4 < 1e-4f) {
        return;
    }
    float t5 = P[16][16]*t2;
    float t6 = P[17][17]*t3;
    float t7 = t2*t2;
    float t8 = R_DECL*t7;
    float t9 = t3*t3;
    float t10 = R_DECL*t9;
    float t11 = R_DECL*t2*t3*2.0f;
    float t14 = P[16][17]*magE*magN;
    float t15 = P[17][16]*magE*magN;
    float t12 = t5+t6+t8+t10+t11-t14-t15;
    float t13;
    if (fabsf(t12) > 1e-6f) {
        t13 = 1.0f / t12;
    } else {
        return;
    }
    float t18 = magE*magE;
    float t19 = magN*magN;
    float t20 = t18+t19;
    float t21;
    if (fabsf(t20) > 1e-6f) {
        t21 = 1.0f/t20;
    } else {
        return;
    }

    // Calculate the observation Jacobian
    // Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
    float H_DECL[24] = {};
    H_DECL[16] = -magE*t21;
    H_DECL[17] = magN*t21;

    Kfusion[0] = -t4*t13*(P[0][16]*magE-P[0][17]*magN);
    Kfusion[1] = -t4*t13*(P[1][16]*magE-P[1][17]*magN);
    Kfusion[2] = -t4*t13*(P[2][16]*magE-P[2][17]*magN);
    Kfusion[3] = -t4*t13*(P[3][16]*magE-P[3][17]*magN);
    Kfusion[4] = -t4*t13*(P[4][16]*magE-P[4][17]*magN);
    Kfusion[5] = -t4*t13*(P[5][16]*magE-P[5][17]*magN);
    Kfusion[6] = -t4*t13*(P[6][16]*magE-P[6][17]*magN);
    Kfusion[7] = -t4*t13*(P[7][16]*magE-P[7][17]*magN);
    Kfusion[8] = -t4*t13*(P[8][16]*magE-P[8][17]*magN);
    Kfusion[9] = -t4*t13*(P[9][16]*magE-P[9][17]*magN);

    if (!inhibitDelAngBiasStates) {
        Kfusion[10] = -t4*t13*(P[10][16]*magE-P[10][17]*magN);
        Kfusion[11] = -t4*t13*(P[11][16]*magE-P[11][17]*magN);
        Kfusion[12] = -t4*t13*(P[12][16]*magE-P[12][17]*magN);
    } else {
        // zero indexes 10 to 12 = 3*4 bytes
        memset(&Kfusion[10], 0, 12);
    }

    if (!inhibitDelVelBiasStates) {
        Kfusion[13] = -t4*t13*(P[13][16]*magE-P[13][17]*magN);
        Kfusion[14] = -t4*t13*(P[14][16]*magE-P[14][17]*magN);
        Kfusion[15] = -t4*t13*(P[15][16]*magE-P[15][17]*magN);
    } else {
        // zero indexes 13 to 15 = 3*4 bytes
        memset(&Kfusion[13], 0, 12);
    }

    if (!inhibitMagStates) {
        Kfusion[16] = -t4*t13*(P[16][16]*magE-P[16][17]*magN);
        Kfusion[17] = -t4*t13*(P[17][16]*magE-P[17][17]*magN);
        Kfusion[18] = -t4*t13*(P[18][16]*magE-P[18][17]*magN);
        Kfusion[19] = -t4*t13*(P[19][16]*magE-P[19][17]*magN);
        Kfusion[20] = -t4*t13*(P[20][16]*magE-P[20][17]*magN);
        Kfusion[21] = -t4*t13*(P[21][16]*magE-P[21][17]*magN);
    } else {
        // zero indexes 16 to 21 = 6*4 bytes
        memset(&Kfusion[16], 0, 24);
    }

    if (!inhibitWindStates) {
        Kfusion[22] = -t4*t13*(P[22][16]*magE-P[22][17]*magN);
        Kfusion[23] = -t4*t13*(P[23][16]*magE-P[23][17]*magN);
    } else {
        // zero indexes 22 to 23 = 2*4 bytes
        memset(&Kfusion[22], 0, 8);
    }

    // get the magnetic declination
    float magDecAng = MagDeclination();

    // Calculate the innovation
    float innovation = atan2f(magE , magN) - magDecAng;

    // limit the innovation to protect against data errors
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the covariance P = (I - K*H)*P
    // take advantage of the empty columns in KH to reduce the
    // number of operations
    for (unsigned i = 0; i<=stateIndexLim; i++) {
        for (unsigned j = 0; j<=15; j++) {
            KH[i][j] = 0.0f;
        }
        KH[i][16] = Kfusion[i] * H_DECL[16];
        KH[i][17] = Kfusion[i] * H_DECL[17];
        for (unsigned j = 18; j<=23; j++) {
            KH[i][j] = 0.0f;
        }
    }
    for (unsigned j = 0; j<=stateIndexLim; j++) {
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            KHP[i][j] = KH[i][16] * P[16][j] + KH[i][17] * P[17][j];
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
            statesArray[j] = statesArray[j] - Kfusion[j] * innovation;
        }
        stateStruct.quat.normalize();

        // record fusion health status
        faultStatus.bad_decl = false;
    } else {
        // record fusion health status
        faultStatus.bad_decl = true;
    }
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// align the NE earth magnetic field states with the published declination
void NavEKF3_core::alignMagStateDeclination()
{
    // don't do this if we already have a learned magnetic field
    if (magFieldLearned) {
        return;
    }

    // get the magnetic declination
    float magDecAng = MagDeclination();

    // rotate the NE values so that the declination matches the published value
    Vector3f initMagNED = stateStruct.earth_magfield;
    float magLengthNE = norm(initMagNED.x,initMagNED.y);
    stateStruct.earth_magfield.x = magLengthNE * cosf(magDecAng);
    stateStruct.earth_magfield.y = magLengthNE * sinf(magDecAng);

    if (!inhibitMagStates) {
        // zero the corresponding state covariances if magnetic field state learning is active
        float var_16 = P[16][16];
        float var_17 = P[17][17];
        zeroRows(P,16,17);
        zeroCols(P,16,17);
        P[16][16] = var_16;
        P[17][17] = var_17;

        // fuse the declination angle to establish covariances and prevent large swings in declination
        // during initial fusion
        FuseDeclination(0.1f);

    }
}

// record a magnetic field state reset event
void NavEKF3_core::recordMagReset()
{
    magStateInitComplete = true;
    if (inFlight) {
        finalInflightMagInit = true;
    }
    // take a snap-shot of the vertical position, quaternion  and yaw innovation to use as a reference
    // for post alignment checks
    posDownAtLastMagReset = stateStruct.position.z;
    quatAtLastMagReset = stateStruct.quat;
    yawInnovAtLastMagReset = innovYaw;
}

/*
  learn magnetometer biases from GPS yaw. Return true if the
  resulting mag vector is close enough to the one predicted by GPS
  yaw to use it for fallback
*/
bool NavEKF3_core::learnMagBiasFromGPS(void)
{
    if (!have_table_earth_field) {
        // we need the earth field from WMM
        return false;
    }
    if (!inFlight) {
        // don't start learning till we've started flying
        return false;
    }

    mag_elements mag_data;
    if (!storedMag.recall(mag_data, imuDataDelayed.time_ms)) {
        // no mag data to correct
        return false;
    }

    // combine yaw with current quaternion to get yaw corrected quaternion
    Quaternion quat = stateStruct.quat;
    if (yawAngDataDelayed.type == 2) {
        Vector3f euler321;
        quat.to_euler(euler321.x, euler321.y, euler321.z);
        quat.from_euler(euler321.x, euler321.y, yawAngDataDelayed.yawAng);
    } else if (yawAngDataDelayed.type == 1) {
        Vector3f euler312 = quat.to_vector312();
        quat.from_vector312(euler312.x, euler312.y, yawAngDataDelayed.yawAng);
    }

    // build the expected body field from orientation and table earth field
    Matrix3f dcm;
    quat.rotation_matrix(dcm);
    Vector3f expected_body_field = dcm.transposed() * table_earth_field_ga;

    // calculate error in field
    Vector3f err = (expected_body_field - mag_data.mag) + stateStruct.body_magfield;

    // learn body frame mag biases
    stateStruct.body_magfield -= err * EK3_GPS_MAG_LEARN_RATE;

    // check if error is below threshold. If it is then we can
    // fallback to magnetometer on failure of external yaw
    float err_length = err.length();

    // we allow for yaw backback to compass if we have had 50 samples
    // in a row below the threshold. This corresponds to 10 seconds
    // for a 5Hz GPS
    const uint8_t fallback_count_threshold = 50;

    if (err_length > EK3_GPS_MAG_LEARN_LIMIT) {
        gps_yaw_fallback_good_counter = 0;
    } else if (gps_yaw_fallback_good_counter < fallback_count_threshold) {
        gps_yaw_fallback_good_counter++;
    }
    bool ok = gps_yaw_fallback_good_counter >= fallback_count_threshold;
    if (ok) {
        // mark mag healthy to prevent a magTimeout when we start using it
        lastHealthyMagTime_ms = imuSampleTime_ms;
    }
    return ok;
}

// Reset states using yaw from EKF-GSF and velocity and position from GPS
bool NavEKF3_core::EKFGSF_resetMainFilterYaw()
{
    // Don't do a reset unless permitted by the EK3_GSF_USE and EK3_GSF_RUN parameter masks
    if ((yawEstimator == nullptr)
        || !(frontend->_gsfUseMask & (1U<<core_index))
        || EKFGSF_yaw_reset_count >= frontend->_gsfResetMaxCount) {
        return false;
    };

    float yawEKFGSF, yawVarianceEKFGSF;
    if (yawEstimator->getYawData(yawEKFGSF, yawVarianceEKFGSF) && is_positive(yawVarianceEKFGSF) && yawVarianceEKFGSF < sq(radians(GSF_YAW_ACCURACY_THRESHOLD_DEG))) {

        // keep roll and pitch and reset yaw
        resetQuatStateYawOnly(yawEKFGSF, yawVarianceEKFGSF);

        // record the emergency reset event
        EKFGSF_yaw_reset_request_ms = 0;
        EKFGSF_yaw_reset_ms = imuSampleTime_ms;
        EKFGSF_yaw_reset_count++;

        if (!use_compass() || AP::compass().get_num_enabled() == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u yaw aligned using GPS",(unsigned)imu_index);
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "EKF3 IMU%u emergency yaw reset",(unsigned)imu_index);
        }

        // Fail the magnetomer so it doesn't get used and pull the yaw away from the correct value
        allMagSensorsFailed = true;

        // record the yaw reset event
        recordYawReset();

        // reset velocity and position states to GPS - if yaw is fixed then the filter should start to operate correctly
        ResetVelocity();
        ResetPosition();

        // reset test ratios that are reported to prevent a race condition with the external state machine requesting the reset
        velTestRatio = 0.0f;
        posTestRatio = 0.0f;

        return true;

    }

    return false;

}

void NavEKF3_core::resetQuatStateYawOnly(float yaw, float yawVariance)
{
    Quaternion quatBeforeReset = stateStruct.quat;
    Vector3f angleErrVarVec = calcRotVecVariances();

    // check if we should use a 321 or 312 Rotation sequence and update the quaternion
    // states using the preferred yaw definition
    stateStruct.quat.inverse().rotation_matrix(prevTnb);
    Vector3f eulerAngles;
    if (fabsf(prevTnb[2][0]) < fabsf(prevTnb[2][1])) {
        // rolled more than pitched so use 321 rotation order
        stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
        stateStruct.quat.from_euler(eulerAngles.x, eulerAngles.y, yaw);
    } else {
        // pitched more than rolled so use 312 rotation order
        eulerAngles = stateStruct.quat.to_vector312();
        stateStruct.quat.from_vector312(eulerAngles.x, eulerAngles.y, yaw);
    }

    // Update the rotation matrix
    stateStruct.quat.inverse().rotation_matrix(prevTnb);
    
    float deltaYaw = wrap_PI(yaw - eulerAngles.z);

    // calculate the change in the quaternion state and apply it to the output history buffer
    Quaternion quat_delta = stateStruct.quat / quatBeforeReset;
    StoreQuatRotate(quat_delta);

    // update the yaw angle variance using the variance of the EKF-GSF estimate
    angleErrVarVec.z = yawVariance;
    zeroRows(P,0,3);
    zeroCols(P,0,3);
    initialiseQuatCovariances(angleErrVarVec);

    // record the yaw reset event
    yawResetAngle += deltaYaw;
    lastYawReset_ms = imuSampleTime_ms;

    // record the yaw reset event
    recordYawReset();

    // clear all pending yaw reset requests
    gpsYawResetRequest = false;
    magYawResetRequest = false;
    
}
