local mavlink_msgs = require("MAVLink/mavlink_msgs")

local STATUSTEXT_ID = mavlink_msgs.get_msgid("STATUSTEXT")

local RATE_HZ = 10
local STATUS_TEXT_DELAY_TIMEOUT_MS = 2000
local status_text_update_ms = uint32_t(0)
local status_text_queue = {}

-- 0 - Emergency, 1 - Alert, 2 - Critical, 3 - Error, 4 - Warning, 5 - Notice, 6 - Info, 7 - Debug
local MAV_MSG_SEVERITY = 6

local msg_map = {}
msg_map[STATUSTEXT_ID] = "STATUSTEXT"

-- initialize MAVLink rx with number of messages, and buffer depth
mavlink:init(5, 10)

-- register message id to receive
mavlink:register_rx_msgid(STATUSTEXT_ID)

function send_statustext(message)
    gcs:send_text(MAV_MSG_SEVERITY, tostring(message["text"]))
    status_text_update_ms = millis()
end

function handle_statustext()
    if next(status_text_queue) == nil then
        return
    end

    local update_time = millis() - status_text_update_ms > STATUS_TEXT_DELAY_TIMEOUT_MS
    if update_time then
        local message = table.remove(status_text_queue, 1)
        send_statustext(message)
    end
end

function update()
    local msg, _ = mavlink:receive_chan()
    if msg then
        local parsed_msg = mavlink_msgs.decode(msg, msg_map)
        if parsed_msg.msgid == STATUSTEXT_ID then
            table.insert(status_text_queue, parsed_msg)
        end
    end

    handle_statustext()
end

local MAV_SEVERITY_DEBUG = 7

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
      gcs:send_text(MAV_SEVERITY_DEBUG, "Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return protected_wrapper, 1000
  end
  return protected_wrapper, math.floor(1000 / RATE_HZ)
end

-- start running update loop
return protected_wrapper()
