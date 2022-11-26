--[[
 test the load function for loading new code from strings
--]]

gcs:send_text(0,"Testing load() method")

local str = [[
function(x,y)
  return math.sin(x) + math.cos(y)
end
]]

local f,errloc,err = load("return " .. str,"TestFunc","t",_ENV)
if not f then
   gcs:send_text(0,string.format("Error %s: %s", errloc, err))
else
   _ENV["TestFunc"] = f()
   gcs:send_text(0, string.format("TestFunc(3,4) -> %f", TestFunc(3,4)))
end
