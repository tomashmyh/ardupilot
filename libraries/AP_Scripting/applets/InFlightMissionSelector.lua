-- Loads one of three mission files whenever the Mission Reset AUX FUNC switch (24)
-- changes position, as long as the vehicle is NOT in AUTO. Lets you change the
-- mission in-flight from a non-AUTO mode (e.g. Loiter).
-- Files missionL.txt / missionM.txt / missionH.txt in the SD card root map to the
-- switch Low / Mid / High positions.
-- Moving the switch while in AUTO leaves the mission unchanged and warns the user;
-- the new selection is applied as soon as the vehicle leaves AUTO.
-- The native "reset mission to first waypoint" action on the high position is
-- unchanged (handled by ArduPilot's AUX function 24 itself).

local MODE_AUTO     = 3   -- Copter AUTO flight mode number
local MODE_AUTO_RTL = 27  -- Copter AUTO_RTL pseudo-mode (mission landing; mission running)

local last_loaded_pos = nil  -- switch position currently reflected in the mission (nil -> load at startup)
local last_warned_pos = nil  -- switch position last warned about while in AUTO (prevents message spam)

local rc_switch = rc:find_channel_for_option(24)  -- AUX FUNC sw for mission restart

if not rc_switch then  -- requires the switch to be assigned in order to run script
  return
end

local function read_mission(file_name)

  -- Open file and read header
  local file = io.open(file_name, "r")
  if not file then
    return true  -- file does not exist: nothing to do, treat as handled
  end
  local header = file:read('l')

  -- check header; leave the current mission untouched if the file is invalid
  if not header or string.find(header, 'QGC WPL 110') ~= 1 then
    file:close()
    gcs:send_text(4, file_name .. ': incorrect format')
    return true
  end

  -- clear any existing mission (fails only if armed and a mission is RUNNING)
  if not mission:clear() then
    file:close()
    gcs:send_text(4, 'MissionSelector: could not clear mission')
    return false  -- mission running; do not consume, retry next tick
  end

  -- read each line and write to mission
  local item = mavlink_mission_item_int_t()
  local index = 0
  while true do
    local data = {}
    for i = 1, 12 do
      data[i] = file:read('n')
      if data[i] == nil then
        if i == 1 then
          gcs:send_text(6, 'loaded mission: ' .. file_name)
          file:close()
          return true -- reached end of file successfully
        else
          mission:clear() -- discard part-loaded mission
          file:close()
          gcs:send_text(4, file_name .. ': failed to read file')
          return true
        end
      end
    end

    item:seq(data[1])
    item:frame(data[3])
    item:command(data[4])
    item:param1(data[5])
    item:param2(data[6])
    item:param3(data[7])
    item:param4(data[8])
    item:x(data[9]*10^7)
    item:y(data[10]*10^7)
    item:z(data[11])

    if not mission:set_item(index, item) then
      mission:clear() -- discard part-loaded mission
      file:close()
      gcs:send_text(4, string.format('%s: failed to set item %i', file_name, index))
      return true
    end
    index = index + 1
  end
end

function update()
  local sw_pos = rc_switch:get_aux_switch_pos()

  if sw_pos ~= last_loaded_pos then            -- switch moved, or first run
    local mode = vehicle:get_mode()
    if mode == MODE_AUTO or mode == MODE_AUTO_RTL then
      if sw_pos ~= last_warned_pos then         -- warn once per new position (no spam)
        gcs:send_text(4, 'MissionSelector: in AUTO - exit AUTO to change mission')
        last_warned_pos = sw_pos
      end
    else
      local filename
      if sw_pos == 0 then
        filename = 'missionL.txt'
      elseif sw_pos == 1 then
        filename = 'missionM.txt'
      else
        filename = 'missionH.txt'
      end
      if read_mission(filename) then
        last_loaded_pos = sw_pos                -- mark this selection as applied
        last_warned_pos = nil
      end
    end
  end

  return update, 1000
end

gcs:send_text(6, "Loaded InFlightMissionSelector.lua")

return update, 5000
