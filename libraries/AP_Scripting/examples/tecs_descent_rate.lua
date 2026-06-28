--[[
   Command a fixed TECS descent rate while flying a LOITER_TO_ALT mission item
   in AUTO mode. Uses the vehicle:set_tecs_descent_rate_override() binding to
   override the TECS height controller with a target sink rate.
--]]

local PARAM_TABLE_KEY = 17
local PARAM_TABLE_PREFIX = "TDR_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

-- TDR_ENABLE: 0 disables the script, 1 enables it
local TDR_ENABLE = bind_add_param('ENABLE', 1, 0)
-- TDR_RATE: demanded descent rate in m/s (positive = descending)
local TDR_RATE   = bind_add_param('RATE',   2, 10)

local MODE_AUTO = 10
local NAV_LOITER_TO_ALT = 31

local UPDATE_MS = 100
-- keep the override active a little longer than one update so it persists
-- between calls but auto-cancels within OVERRIDE_MS if the script stops
local OVERRIDE_MS = 500

local last_rate = nil

local function update()
   if TDR_ENABLE:get() == 0 then
      return
   end

   -- only act in AUTO mode while running a LOITER_TO_ALT mission item
   if vehicle:get_mode() ~= MODE_AUTO or
      mission:get_current_nav_id() ~= NAV_LOITER_TO_ALT then
      last_rate = nil
      return
   end

   local rate = TDR_RATE:get()
   if vehicle:set_tecs_descent_rate_override(rate, OVERRIDE_MS) then
      -- report on activation and whenever the demanded rate changes
      if rate ~= last_rate then
         gcs:send_text(6, string.format("TDR: descent rate %.1f m/s", rate))
         last_rate = rate
      end
   else
      last_rate = nil
   end
end

local function loop()
   update()
   return loop, UPDATE_MS
end

gcs:send_text(6, "TDR: loaded TECS descent rate control")
return loop, UPDATE_MS
