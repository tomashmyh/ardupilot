-- RC driven compass calibrating

-- Settings
local CH = 10
local START_PWM = 1750

local PWM_DEVIATION = 100

-- Constants. Don`t change.
local START_PWM_MAX = START_PWM + PWM_DEVIATION
local START_PWM_MIN = START_PWM - PWM_DEVIATION

local COMPASS_CALIBRATION_AUX_FUNC = 171
local HIGH_AUX_LEVEL = 2
local LOW_AUX_LEVEL = 0

local aux_level = LOW_AUX_LEVEL

local MAV_WARN = 4

function update()
   rc_input = rc:get_pwm(CH)

   if (rc_input > START_PWM_MIN) and (rc_input < START_PWM_MAX) then
      if aux_level ~= HIGH_AUX_LEVEL then
         aux_level = HIGH_AUX_LEVEL
         rc:run_aux_function(COMPASS_CALIBRATION_AUX_FUNC, HIGH_AUX_LEVEL)
         gcs:send_text(MAV_WARN, "Compass calibration started")
      end
   else
      if aux_level ~= LOW_AUX_LEVEL then
         aux_level = LOW_AUX_LEVEL
         rc:run_aux_function(COMPASS_CALIBRATION_AUX_FUNC, LOW_AUX_LEVEL)
         gcs:send_text(MAV_WARN, "Compass calibration stopped")
      end
   end
   return update, 1000
end

return update()
