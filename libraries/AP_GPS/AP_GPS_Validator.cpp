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

#include "AP_GPS_config.h"

#if AP_GPS_ENABLED

#include <stdint.h>

#include "AP_GPS.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_GPS::AP_GPS_Validator::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable GPS validator
    // @Description: Controls if GPS validation is enabled
    // @Values: 0:No,1:Yes
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_GPS::AP_GPS_Validator, is_enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ACTION
    // @DisplayName: Action on GPS validation failure
    // @Description: Defines an action involved when GPS identified as a bad
    // @Values: 0:OnlyInform,1:OnlyDisableGPSUse,1:InformAndDisableGPSUse
    // @User: Advanced
    AP_GROUPINFO("ACTION", 2, AP_GPS::AP_GPS_Validator, action_on_failure, static_cast<int8_t>(AP_GPS_Validator::Action::ONLY_INFORM)),

    // @Param: SAT_N
    // @DisplayName: Minimum satellites number
    // @Description: Defines the minimum satellites count to consider GPS as a good
    // @User: Advanced
    AP_GROUPINFO("SAT_N", 3, AP_GPS::AP_GPS_Validator, min_sat_count, 6),

    // @Param: V_H_MAX
    // @DisplayName: Maximum horizontal velocity
    // @Description: Defines the maximum horizontal velocity in meters per seconds to consider GPS as a good
    // @Units: mps
    // @User: Advanced
    AP_GROUPINFO("V_H_MAX", 4, AP_GPS::AP_GPS_Validator, max_horizontal_speed_mps, 30),

    // @Param: V_V_MAX
    // @DisplayName: Maximum vertical velocity
    // @Description: Defines the maximum vertical velocity in meters per seconds to consider GPS as a good
    // @Units: mps
    // @User: Advanced
    AP_GROUPINFO("V_V_MAX", 5, AP_GPS::AP_GPS_Validator, max_vertical_speed_mps, 15),

    // @Param: ALT_MAX
    // @DisplayName: Maximum allowed MSL altitude
    // @Description: Defines the maximum allowed altitude in meters to consider GPS as a good
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ALT_MAX", 6, AP_GPS::AP_GPS_Validator, max_allowed_alt_m, 2000),

    // @Param: ALT_MIN
    // @DisplayName: Minimum allowed MSL altitude
    // @Description: Defines the minimum allowed altitude in meters to consider GPS as a good
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ALT_MIN", 7, AP_GPS::AP_GPS_Validator, min_allowed_alt_m, -10),

    // @Param: TIME_A
    // @DisplayName: Time accuracy
    // @Description: Defines time accuracy in miliseconds to consider GPS as a good
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("TIME_A", 8, AP_GPS::AP_GPS_Validator, time_accuracy_ms, 10),

    // @Param: INST
    // @DisplayName: GPS instance to validate
    // @Description: Defines a GPS instance to run the gps validation on
    // @Values: 0:First,1:Second,1:Primary
    // @User: Advanced
    AP_GROUPINFO("INST", 9, AP_GPS::AP_GPS_Validator, gps_instance_to_validate, static_cast<int8_t>(AP_GPS_Validator::GpsInstance::FIRST)),

    AP_GROUPEND
};

AP_GPS::AP_GPS_Validator::AP_GPS_Validator()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_GPS::AP_GPS_Validator::apply_enable_state(bool enabled) {
    is_enabled.set_enable(enabled);
}

void AP_GPS::AP_GPS_Validator::change_action_on_failure(AP_GPS_Validator::Action action) {
    action_on_failure.set(static_cast<int8_t>(action));
}

uint32_t AP_GPS::AP_GPS_Validator::now_ms() const {
    return AP_HAL::millis();
}

bool AP_GPS::AP_GPS_Validator::trust_gps(const AP_GPS::GPS_State& state) {
    if (!is_enabled) {
        return true;
    }

    const auto desired_instance = gps_instance_to_validate.get();

    if (state.instance != desired_instance && desired_instance != static_cast<int8_t>(GpsInstance::PRIMARY)) {
        return true;
    }

    const auto action = get_gps_failure_action();
    const bool inform = should_inform(action);

    const uint32_t now = now_ms();

    // Evaluate current sample (first failure reason)
    const FailureReason fail = first_failure_reason(state, now);
    const bool candidate_good = (fail == FailureReason::NONE);

    // Update sample bookkeeping (used by speed/time checks next call)
    last_state = state;
    last_gps_time_ms = now;

    if (!candidate_good) {
        // become bad immediately
        if (is_gps_good) {
            // State change true -> false: send exactly one message (first detected failure)
            if (inform) {
                send_failure_text(fail, state.instance + 1);
            }
            is_gps_good = false;
        }
        // Cancel any pending recovery timer
        pending_good_valid = false;

    } else {
        // candidate_good == true
        if (!is_gps_good) {
            // recover only after debounce
            if (!pending_good_valid) {
                pending_good_valid = true;
                pending_good_since_ms = now;
            }

            const uint32_t elapsed_ms = now - pending_good_since_ms;
            if (elapsed_ms >= (CHANGE_STATE_DELAY_S * 1000U)) {
                // Commit false -> true
                is_gps_good = true;
                pending_good_valid = false;

                // message only on state change
                if (inform) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d: good", state.instance + 1);
                }
            }
        } else {
            // Already good; nothing pending
            pending_good_valid = false;
        }
    }

    // only ONLY_INFORM gates the return value
    return (action != Action::ONLY_INFORM) ? is_gps_good : true;
}

bool AP_GPS::AP_GPS_Validator::is_satellites_ok(const AP_GPS::GPS_State& state) const {
    return state.num_sats >= min_sat_count.get();
}

bool AP_GPS::AP_GPS_Validator::is_horizontal_speed_ok(const AP_GPS::GPS_State& state, uint32_t now_ms) const {
    const int32_t dt_ms = int32_t(state.last_gps_time_ms - last_state.last_gps_time_ms);
    if (dt_ms <= 0) {
        return true;
    }

    const float time_diff_s = dt_ms * 0.001f;

    const auto horizontal_distance_m = static_cast<float>(state.location.get_distance(last_state.location));

    const float horizontal_speed_mps = fabsf(horizontal_distance_m) / time_diff_s;

    return horizontal_speed_mps <= max_horizontal_speed_mps;
}

bool AP_GPS::AP_GPS_Validator::is_vertical_speed_ok(const AP_GPS::GPS_State& state, uint32_t now_ms) const {
    const float time_diff_s = (state.last_gps_time_ms - last_state.last_gps_time_ms) * 0.001f;
    if (time_diff_s <= 0) {
        return true;
    }

    ftype altitude_diff_m = 0.0;
    if (!state.location.get_alt_distance(last_state.location, altitude_diff_m)) {
        return false;
    }

    const float vertical_speed_mps = fabsf(static_cast<float>(altitude_diff_m)) / time_diff_s;

    return vertical_speed_mps <= max_vertical_speed_mps;
}

bool AP_GPS::AP_GPS_Validator::is_altitude_ok(const AP_GPS::GPS_State& state) const {
    const float altitude_m = state.location.alt * 0.01;

    return altitude_m >= min_allowed_alt_m.get() && altitude_m <= max_allowed_alt_m.get();
}

bool AP_GPS::AP_GPS_Validator::is_time_ok(const AP_GPS::GPS_State& state, uint32_t now_ms) const {
    if (last_gps_time_ms == UINT32_MAX) {
        return false; // no prior sample
    }
    const uint32_t dt_ms = now_ms - last_gps_time_ms;
    return dt_ms >= static_cast<uint32_t>(time_accuracy_ms.get());
}

AP_GPS::AP_GPS_Validator::Action AP_GPS::AP_GPS_Validator::get_gps_failure_action() const {
    const auto action = action_on_failure.get();
    if (action > static_cast<int8_t>(Action::LAST) || action < static_cast<int8_t>(Action::FIRST)) {
        return Action::FIRST;
    }
    return static_cast<Action>(action);
}

AP_GPS::AP_GPS_Validator::FailureReason AP_GPS::AP_GPS_Validator::first_failure_reason(const AP_GPS::GPS_State& state, uint32_t now_ms) const
{
    if (!is_satellites_ok(state)) {
        return FailureReason::SATS;
    }
    if (!is_time_ok(state, now_ms)) {
        return FailureReason::TIME;
    }
    if (!is_horizontal_speed_ok(state, now_ms)) {
        return FailureReason::HSPEED;
    }
    if (!is_vertical_speed_ok(state, now_ms)) {
        return FailureReason::VSPEED;
    }
    if (!is_altitude_ok(state)) {
        return FailureReason::ALT;
    }
    return FailureReason::NONE;
}

void AP_GPS::AP_GPS_Validator::send_failure_text(AP_GPS::AP_GPS_Validator::FailureReason reason, uint8_t gps_instance_plus1)
{
    switch (reason) {
    case AP_GPS::AP_GPS_Validator::FailureReason::SATS:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: bad sats", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::HSPEED:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: bad hspeed", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::VSPEED:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: bad vspeed", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::ALT:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: bad alt", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::TIME:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: bad time", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::NONE:
    default:
        break;
    }
}

bool AP_GPS::AP_GPS_Validator::should_inform(Action action) {
    return action != Action::ONLY_DISABLE_GPS_USE;
}

#endif  // AP_GPS_ENABLED
