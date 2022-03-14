local loiter_exit_alt_cm = 300000 -- AMSL height in cm where we exit loiter down and go into GUIDED mode
local turn_separation_time_ms = 15000 -- time between turns in guided mode
local stabilize_exit_sink_min = 10.0 -- minimum sink rate in m/s allowed for exit from STABILIZE
local stabilize_exit_bank_max = 20.0 -- maximum roll angle in degrees allowed for exit from STABILIZE
local stabilize_angle_rate_max = 40.0 -- maximum angle rate in degrees/sec allowed for exit from STABILIZE

local manoeuvre_start_time_ms = 0
local target_bearing = 0
local last_turn_right = false
local gps_loc

function update()
    -- If aircraft falling and stable - switch to loiter
    local roll_angle_deg = math.deg(ahrs:get_roll())
    local angle_rate_vec = Vector3f()
    angle_rate_vec = ahrs:get_gyro()
    local angle_rate_mag = math.sqrt(angle_rate_vec:x()*angle_rate_vec:x() + angle_rate_vec:y()*angle_rate_vec:y() + angle_rate_vec:z()*angle_rate_vec:z())
    local velocity_NED = ahrs:get_velocity_NED()
    gps_loc = gps:location(0)
    gps_status = gps:status(0)
    time_ms = millis()

    -- handle transition from STABiLIZE to LOITER
    if ((vehicle:get_mode() == 2) and (math.abs(roll_angle_deg) < stabilize_exit_bank_max) and (math.deg(angle_rate_mag) < stabilize_angle_rate_max) and (velocity_NED:z() > stabilize_exit_sink_min)) then
        vehicle:set_mode(12)
    end

    -- allow to loiter down 1500m and fly out to target location on a 3:1 glide ratio for 5000m
    if ((gps_loc ~= nil) and (gps_status >= 3) and (vehicle:get_mode() == 12) and gps_loc:alt() < loiter_exit_alt_cm) then
        target_bearing = math.atan(velocity_NED:y(),velocity_NED:x())
        local delta_north = 5000.0 * math.cos(target_bearing)
        local delta_east = 5000.0 * math.sin(target_bearing)
        local delta_down = 5000.0/3.5
        local target_loc = gps_loc
        target_loc:offset(delta_north,delta_east)
        local new_alt_cm = target_loc:alt() - math.floor(delta_down * 100.0)
        target_loc:alt(new_alt_cm)
        vehicle:set_mode(15)
        vehicle:set_target_location(target_loc)
        manoeuvre_start_time_ms = time_ms
    end

    -- do a continuous series of RH and LH 90 degree turns at 15 second intervals
    if ((vehicle:get_mode() == 15) and (gps_status >= 3) and (time_ms - manoeuvre_start_time_ms > turn_separation_time_ms)) then
        if (last_turn_right) then
            target_bearing = target_bearing - 0.5 * math.pi
            last_turn_right = false
        else
            target_bearing = target_bearing + 0.5 * math.pi
            last_turn_right = true
        end
        local delta_north = 5000.0 * math.cos(target_bearing)
        local delta_east = 5000.0 * math.sin(target_bearing)
        local delta_down = 5000.0/3.5
        local target_loc = gps_loc
        target_loc:offset(delta_north,delta_east)
        local new_alt_cm = target_loc:alt() - math.floor(delta_down * 100.0)
        target_loc:alt(new_alt_cm)
        vehicle:set_target_location(target_loc)
        manoeuvre_start_time_ms = time_ms
    end

      -- run at 5Hz
  return update, 200

end

return update()