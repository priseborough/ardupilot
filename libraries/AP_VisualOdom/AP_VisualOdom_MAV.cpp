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

#include "AP_VisualOdom_MAV.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// consume vision position estimate data and send to EKF. distances in meters
void AP_VisualOdom_MAV::handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms,
                                                        const Vector3f position,
                                                        const Vector3f rpy,
                                                        const float covariance[21],
                                                        uint8_t reset_counter)
{
    const float scale_factor =  _frontend.get_pos_scale();
    Vector3f pos = position * scale_factor;

    // send attitude and position to EKF
    AP::ahrs().writeExtNavData(pos, rpy, covariance, time_ms, _frontend.get_delay_ms(), get_reset_timestamp_ms(reset_counter));

    // log sensor data
    const float posErr = (covariance[0]+covariance[6]+covariance[11]) / 3.0f;
    const float angErr = (covariance[15]+covariance[18]+covariance[20]) / 3.0f;
    Write_VisualPosition(remote_time_us, time_ms, pos.x, pos.y, pos.z, rpy[0], rpy[1], rpy[2], posErr, angErr, reset_counter, false);

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();
}

void AP_VisualOdom_MAV::handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter)
{
    // send velocity to EKF
    AP::ahrs().writeExtNavVelData(vel, _frontend.get_vel_noise(), time_ms, _frontend.get_delay_ms());

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();

    Write_VisualVelocity(remote_time_us, time_ms, vel, reset_counter, false);
}

#endif
