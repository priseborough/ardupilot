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
#pragma once

/*
 *       OpticalFlow.h - OpticalFlow Base Class for Ardupilot
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class OpticalFlow_backend;
class AP_AHRS_NavEKF;

class OpticalFlow
{
    friend class OpticalFlow_backend;

public:
    static OpticalFlow create(AP_AHRS_NavEKF& ahrs) { return OpticalFlow{ahrs}; }

    constexpr OpticalFlow(OpticalFlow &&other) = default;

    /* Do not allow copies */
    OpticalFlow(const OpticalFlow &other) = delete;
    OpticalFlow &operator=(const OpticalFlow&) = delete;

    // init - initialise sensor
    void init(void);

    // enabled - returns true if optical flow is enabled
    bool enabled() const { return _enabled; }

    // healthy - return true if the sensor is healthy
    bool healthy() const { return backend != nullptr && _flags.healthy; }

    // read latest values from sensor and fill in x,y and totals.
    void update(void);

    // quality - returns the surface quality as a measure from 0 ~ 255
    uint8_t quality() const { return _state.surface_quality; }

    // Return optical flow angular rate in rad/sec measured about the X and Y sensor axis. A RH rotation about a sensor axis produces a positive rate.
    // This rate will be the combination of translational and rotation movement of the sensor relative to it's surroundings.
    const Vector2f& flowRate() const { return _state.flowRate; }

    // Return body inertial angular rate in rad/sec measured about the X and Y sensor axis. A RH rotation about a sensor axis produces a positive rate.
    const Vector2f& bodyRate() const { return _state.bodyRate; }

    // device_id - returns device id
    uint8_t device_id() const { return _state.device_id; }

    // last_update() - returns system time of last sensor update
    uint32_t last_update() const { return _last_update_ms; }

    struct OpticalFlow_state {
        uint8_t device_id;          // device id
        uint8_t  surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
        Vector2f flowRate;          // optical flow angular rate in rad/sec measured about the X and Y sensor axis. A RH rotation about a sensor axis produces a positive rate.
        Vector2f bodyRate;          // body inertial angular rate in rad/sec measured about the X and Y sensor axis. A RH rotation about a sensor axis produces a positive rate.
    };

    // return a 3D vector defining the position offset of the sensors focal point in metres relative to the body frame origin
    const Vector3f &get_pos_offset(void) const {
        return _pos_offset;
    }

    // Return a 3x3 matrix  defining the transformation of a vector from body to sensor frame of reference.
    const Matrix3f &get_sensor_rotmat(void) const {
        return _sensor_rotmat;
    }

    // parameter var info table
    static const struct AP_Param::GroupInfo var_info[];

private:
    OpticalFlow(AP_AHRS_NavEKF& ahrs);

    AP_AHRS_NavEKF &_ahrs;
    OpticalFlow_backend *backend;

    struct AP_OpticalFlow_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
    } _flags;

    // parameters
    AP_Int8  _enabled;              // enabled/disabled flag
    AP_Int16 _flowScalerX;          // X axis flow scale factor correction - parts per thousand
    AP_Int16 _flowScalerY;          // Y axis flow scale factor correction - parts per thousand
    AP_Vector3f _pos_offset;        // position offset of the flow sensor in the body frame
    AP_Int8  _address;              // address on the bus (allows selecting between 8 possible I2C addresses for px4flow)
    AP_Int16 _rot_x_cd;             // X-axis rotation from a ZYX Tait-Bryan rotation sequence defining the rotation from body frame to sensor frame (centi-deg)
    AP_Int16 _rot_y_cd;             // Y-axis rotation from a ZYX Tait-Bryan rotation sequence defining the rotation from body frame to sensor frame (centi-deg)
    AP_Int16 _rot_z_cd;             // Z-axis rotation from a ZYX Tait-Bryan rotation sequence defining the rotation from body frame to sensor frame (centi-deg)

    Matrix3f _sensor_rotmat;        // rotation matrix from body frame to sensor frame

    // state filled in by backend
    struct OpticalFlow_state _state;

    uint32_t _last_update_ms;        // millis() time of last update
};

#include "OpticalFlow_backend.h"
