-- This script is a test of led override

local count = 0
local total_time = 1
local animation_end = 0
local current_anim = 0
local brightness = 0

-- the table key must be used by only one script on a particular flight
-- controller. If you want to re-use it then you need to wipe your old parameters
-- the key must be a number between 0 and 200. The key is persistent in storage
local PARAM_TABLE_KEY = 72
local PARAM_TABLE_PREFIX = "ANIM_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), 'could not ANIM_ add param table')

function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
  return Parameter(PARAM_TABLE_PREFIX .. name)
end
local enable = bind_add_param('ENABLE', 1, 1)
-- constrain a value between limits
function constrain(v, vmin, vmax)
  if v < vmin then
     v = vmin
  end
  if v > vmax then
     v = vmax
    
  end
  return v
end

--[[
Table of neon colors on a rainbow, red first
--]]
-- local rainbow = {
--   { 252, 23, 0 },
--   { 253, 72, 37 },
--   { 250, 237, 39 },
--   { 51,   255, 20 },
--   { 27,   3,   163 }
-- }

local rainbow = {
  { 252, 23, 0 },
  { 253, 100, 0 },
  { 250, 237, 0 },
  { 51,   255, 20 },
  { 27,   3,   163 }
}
-- 0-49
local led_map = {
  37,
  38,
  39,
  40,
  41,
  42,
  43,
  44,
  45,
  46,
  47,
  48,
  49,
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  13,
  14,
  15,
  16,
  17,
  18,
  19,
  20,
  21,
  22,
  23,
  24,
  25,
  26,
  27,
  28,
  29,
  30,
  31,
  32,
  33,
  34,
  35,
  36,
  }

local led_len = 50

function table.contains(table, element)
  for _, value in pairs(table) do
    if value == element then
      return true
    end
  end
  return false
end

--[[
Function to send a color to a LED
--]]
function set_LED(r, g, b, id)
  serialLED:set_RGB(1, id, math.floor(r*brightness/100), math.floor(g*brightness/100), math.floor(b*brightness/100))
end

--[[
Function to set a LED to a color on a classic rainbow spectrum, with v=0 giving red
--]]
function get_Rainbow(v, num_rows)
  local row = math.floor(constrain(v * (num_rows-1)+1, 1, num_rows-1))
  local v0 = (row-1) / (num_rows-1)
  local v1 = row / (num_rows-1)
  local p = (v - v0) / (v1 - v0)
  r = math.max(math.floor(rainbow[row][1] + p * (rainbow[row+1][1] - rainbow[row][1])),0)
  g = math.max(math.floor(rainbow[row][2] + p * (rainbow[row+1][2] - rainbow[row][2])),0)
  b = math.max(math.floor(rainbow[row][3] + p * (rainbow[row+1][3] - rainbow[row][3])),0)
  return r,g,b
end

function do_initialisation()
  local pos = math.rad(total_time/2)
  local v  =0.5 + 0.5 * math.sin(pos)
  local invert = 1
  local r, g, b = get_Rainbow(v, 5)
  local led_trail_length = 10
  local softness = 10
  local bfact = 1
  local updated_leds = {}
  if math.cos(pos) > 0 then
    invert = 1
  else
    invert = 0
  end

  pos = math.floor(6 + (v*(led_len/2)))%led_len
  if pos == 6 then
    animation_end = 1
  else
    animation_end = 0
  end

  for trail = 0, led_trail_length-1 do
    if invert == 1 then
      bfact = softness^(2*(led_trail_length - 1 - trail)/(led_trail_length-1))
    else
      bfact = softness^(2*trail/(led_trail_length-1))
    end
    local led_id = 1 + ((pos + trail) % led_len)
    set_LED(math.floor(r/bfact), math.floor(g/bfact), math.floor(b/bfact), led_map[led_id])
    table.insert(updated_leds, led_id)
  end

  pos = math.floor(6 - (v*(led_len/2)))%led_len
  for trail = 0, led_trail_length-1 do
    if invert == 0 then
      bfact = softness^(2*(led_trail_length - 1 - trail)/(led_trail_length-1))
    else
      bfact = softness^(2*trail/(led_trail_length-1))
    end
    local led_id = 1 + ((pos + trail) % led_len)
    set_LED(math.floor(r/bfact), math.floor(g/bfact), math.floor(b/bfact), led_map[led_id])
    table.insert(updated_leds, led_id)
  end

  for led = 1, led_len do
    if not table.contains(updated_leds, led) then
      set_LED(0, 0, 0, led_map[led])
    end
  end
end

function do_point_north(r, g, b, led_trail_length, softness)
  local yaw = (led_len * periph:get_yaw_earth()/(2*math.pi))
  yaw = yaw - (led_trail_length/2)
  local ofs = yaw - math.floor(yaw+0.5)
  yaw = math.floor(yaw+0.5) % led_len
  local updated_leds = {}
  for trail = 0, led_trail_length-1 do
    local bfact = softness^(2*(math.abs((led_trail_length-1)/2 - trail + ofs))/(led_trail_length-1))
    local led_id = 1 + ((yaw + trail) % led_len)
    set_LED(math.floor(r/bfact), math.floor(g/bfact), math.floor(b/bfact), led_map[led_id])
    table.insert(updated_leds, led_id)
  end

  -- also do LEDs on the other side of the circle
  yaw = (yaw + ((led_len)/2)) % led_len
  for trail = 0, led_trail_length-1 do
    local bfact = softness^(2*(math.abs((led_trail_length-1)/2 - trail + ofs))/(led_trail_length-1))
    local led_id = 1 + ((yaw + trail) % led_len)
    set_LED(math.floor(r/bfact), math.floor(g/bfact), math.floor(b/bfact), led_map[led_id])
    table.insert(updated_leds, led_id)
  end
  for led = 1, led_len do
    if not table.contains(updated_leds, led) then
      set_LED(0, 0, 0, led_map[led])
    end
  end
  return yaw
end

function finish_initialisation(r, g, b, led_trail_length, softness, speed_factor, next_call)
  local north_bias = 4
  local yaw = (led_len/2) - ((led_len * periph:get_yaw_earth()/(2*math.pi))) - north_bias
  yaw = math.floor(yaw+0.5) % led_len
  local led_id = math.floor((count/speed_factor) + north_bias) % led_len
  if yaw == led_id then
    if total_time > 3000 then
      animation_end = 1
    end
    return
  end
  count = count + next_call
  local updated_leds = {}
  for trail = 0, led_trail_length-1 do
    local bfact = softness^(2*(math.abs((led_trail_length-1)/2 - trail))/(led_trail_length-1))
    local this_led_id = 1 + ((led_id + trail) % led_len)
    set_LED(math.floor(r/bfact), math.floor(g/bfact), math.floor(b/bfact), led_map[this_led_id])
    table.insert(updated_leds, this_led_id)
  end

  for led = 1, led_len do
    if not table.contains(updated_leds, led) then
      set_LED(0, 0, 0, led_map[led])
    end
  end
end

function do_arm_spin(r, g, b, softness, speed_factor, arming)
  -- reset led states
  local updated_leds = {}

  -- calculate next call based on time
  local next_call = 1
  if arming then
    next_call = speed_factor - math.floor(((total_time)/speed_factor) + 0.5)
    if next_call < 1 then
      return next_call
    end
  else
    next_call = 1 + math.floor(((total_time)/speed_factor) + 0.5)
    if next_call > speed_factor then
      return next_call
    end
  end
  -- set trail length bassed on spin progress
  local led_trail_length =  3 + (13 * (1 - (next_call/75)))


  -- create a trail
  for trail = 0, led_trail_length-1 do
    local bfact = softness^(led_trail_length - trail)
    local led_id = 1 + (count + trail) % led_len
    set_LED(math.floor(r/bfact), math.floor(g/bfact), math.floor(b/bfact), led_map[led_id])
    table.insert(updated_leds, led_id)
  end

  for led = 1, led_len do
    if not table.contains(updated_leds, led) then
      set_LED(0, 0, 0, led_map[led])
    end
  end
  return next_call
end

function set_all(r, g, b)
  for led = 1, led_len do
    set_LED(r, g, b, led_map[led])
  end
end

function animation_state_machine(vehicle_state)
  -- change state only when last loop finished
  if animation_end == 0 then
    return
  end

  if ((vehicle_state & 1) == uint32_t(1)) then
    -- do_initialisation
    if current_anim ~= 0 then
      current_anim = 0
      total_time = 0
      animation_end = 0
    end
  elseif current_anim == 0 then
    -- do finish by move to north
    count = 0
    total_time = 0
    current_anim = 1
    animation_end = 0
  elseif current_anim == 1 then
    -- do always point north
    current_anim = 2
  elseif current_anim ~= 3 and ((vehicle_state & (1 << 1)) ~= uint32_t(0)) then --VEHICLE_STATE_ARMED
    total_time = 0
    current_anim = 3
    animation_end = 0
  elseif current_anim == 3 and ((vehicle_state & (1 << 1)) == uint32_t(0)) then
    total_time = 0
    current_anim = 4
    animation_end = 0
  elseif current_anim == 4 then
    current_anim = 1
  end
end

function update() -- this is the loop which periodically runs
  if enable:get() == 0 then
    return update, 1000
  end
  local next_call = 20
  local vehicle_state = periph:get_vehicle_state()
  animation_state_machine(vehicle_state)
  -- get parameter NTF_LED_BRIGHT
  brightness = param:get('NTF_LED_BRIGHT')

  -- switch case for brightness
  if brightness == 0 then
    brightness = 0
  elseif brightness == 1 then
    brightness = 20
  elseif brightness == 2 then
    brightness = 50
  elseif brightness == 3 then
    brightness = 100
  end

  -- Initialisation
  if current_anim == 0 then
    do_initialisation()
    total_time = total_time + next_call
  elseif current_anim == 1 then
    finish_initialisation(rainbow[1][1], rainbow[1][2], rainbow[1][3], 5, 20, 50, next_call)
    total_time = total_time + next_call
  elseif current_anim == 2 then
    local v
    if (vehicle_state & (1<<3)) ~= uint32_t(0) or --VEHICLE_STATE_PREARM
       (vehicle_state & (1<<4)) ~= uint32_t(0) or --VEHICLE_STATE_PREARM_GPS
       (vehicle_state & (1<<7)) ~= uint32_t(0) or --VEHICLE_STATE_FAILSAFE_RADIO
       (vehicle_state & (1<<8)) ~= uint32_t(0) or --VEHICLE_STATE_FAILSAFE_BATT
       (vehicle_state & (1<<11)) ~= uint32_t(0) then --VEHICLE_STATE_EKF_BAD
      v = 0.0
    end
    if (vehicle_state & (1<<17)) == uint32_t(0) then--VEHICLE_STATE_POS_ABS_AVAIL
      v = 0.5 + (0.5 * math.min(gps:num_sats(0)/20, 1.0))
    else
      v = 1.0
    end
    local r, g, b = get_Rainbow(v, 4)
    count = do_point_north(r, g, b, 10, 20)
  -- --[[ ARM Display
  elseif current_anim == 3 then
    if animation_end == 0 then
      next_call = do_arm_spin(rainbow[4][1], rainbow[4][2], rainbow[4][3], 2, 75, true)
      count = count + 1
    end
    if next_call < 1 then
      set_all(rainbow[4][1], rainbow[4][2], rainbow[4][3])
      animation_end = 1
    else
      total_time = total_time + next_call
    end
  elseif current_anim == 4 then
    if animation_end == 0 then
      next_call = do_arm_spin(rainbow[4][1], rainbow[4][2], rainbow[4][3], 2, 75, false)
      count = count - 1
    end
    if next_call > 75 then
      animation_end = 1
    else
      total_time = total_time + next_call
    end
  end
  serialLED:send(1)
  -- gcs:send_text(0, string.format("NCALL: %s", tostring(next_call)))
  return update, next_call -- reschedules the loop in next_call milliseconds
end

return update() -- run immediately before starting to reschedule
