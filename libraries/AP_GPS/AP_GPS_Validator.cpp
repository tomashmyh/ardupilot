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
    // @Values: 0:DoNotInform,1:OnlyInform,1:DisableGPSUse
    // @User: Advanced
    AP_GROUPINFO("ACTION", 2, AP_GPS::AP_GPS_Validator, action_on_failure, static_cast<int8_t>(AP_GPS_Validator::Action::DO_NOT_INFROM)),

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

    // @Param: TIME_ACR
    // @DisplayName: Time accuracy
    // @Description: Defines time accuracy in miliseconds to consider GPS as a good
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("TIME_A", 8, AP_GPS::AP_GPS_Validator, time_accuracy_ms, 10),

    AP_GROUPEND
};

AP_GPS::AP_GPS_Validator::AP_GPS_Validator() : last_state{},
                                               last_gps_time_us(UINT64_MAX),
                                               last_gps_state_change_us(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_GPS::AP_GPS_Validator::trust_gps(const AP_GPS::GPS_State& state) {
    // We validate only GPS that have 2D and higher fix. All other statuses are properly handled by ardupilot
    if (!is_enabled || state.status < AP_GPS::GPS_OK_FIX_2D) {
        return true;
    }

    const auto action = get_gps_failure_action();

    const uint64_t gps_time_us = AP::gps().time_epoch_usec(state);
    const uint64_t gps_state_change_diff_s = (gps_time_us - last_gps_state_change_us) * 1e-6;

    const auto inform = should_inform(action);

    const bool was_gps_good = is_gps_good;

    bool is_ok = is_satellites_ok(state);
    if (!is_ok && was_gps_good && inform) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: not enough satellites", state.instance + 1);
    }
    is_gps_good = is_ok;

    is_ok = is_horizontal_speed_ok(state);
    if (!is_ok && was_gps_good && is_gps_good && inform) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: horizontal speed failure", state.instance + 1);
    }
    is_gps_good &= is_ok;

    is_ok = is_vertical_speed_ok(state);
    if (!is_ok && was_gps_good && is_gps_good && inform) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: vertical speed failure", state.instance + 1);
    }
    is_gps_good &= is_ok;

    is_ok = is_altitude_ok(state);
    if (!is_ok && was_gps_good && is_gps_good && inform) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: altitude failure", state.instance + 1);
    }
    is_gps_good &= is_ok;

    is_ok = is_time_ok(state, gps_time_us);
    if (!is_ok && was_gps_good && is_gps_good && inform) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: time accuracy failure", state.instance + 1);
    }
    is_gps_good &= is_ok;

    last_state = state;
    last_gps_time_us = gps_time_us;

    if (is_gps_good && !was_gps_good && inform) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d is good for flight", state.instance + 1);
    }

    const auto should_change_state = gps_state_change_diff_s > CHANGE_STATE_DELAY_S && is_gps_good;

    if (should_change_state) {
        last_gps_state_change_us = gps_time_us;
    } else {
        is_gps_good = was_gps_good;
    }

    return (action == Action::DISABLE_GPS_USE) ? is_gps_good : true;
}

bool AP_GPS::AP_GPS_Validator::is_satellites_ok(const AP_GPS::GPS_State& state) const {
    return state.num_sats >= min_sat_count.get();
}

bool AP_GPS::AP_GPS_Validator::is_horizontal_speed_ok(const AP_GPS::GPS_State& state) const {
    const float time_diff_s = (state.last_gps_time_ms - last_state.last_gps_time_ms) * 0.001;
    if (time_diff_s <= 0) {
      return true;
    }

    const ftype horizontal_distance_m = state.location.get_distance(last_state.location);

    const float horizontal_speed_mps = abs(horizontal_distance_m) / time_diff_s;

    return horizontal_speed_mps <= max_horizontal_speed_mps;
}

bool AP_GPS::AP_GPS_Validator::is_vertical_speed_ok(const AP_GPS::GPS_State& state) const {
    const float time_diff_s = (state.last_gps_time_ms - last_state.last_gps_time_ms) * 0.001;
    if (time_diff_s <= 0) {
      return true;
    }

    ftype altitude_diff_m = 0.0;
    if (!state.location.get_alt_distance(last_state.location, altitude_diff_m)) {
      return false;
    }

    const float vertical_speed_mps = abs(altitude_diff_m) / time_diff_s;

    return vertical_speed_mps <= max_vertical_speed_mps;
}

bool AP_GPS::AP_GPS_Validator::is_altitude_ok(const AP_GPS::GPS_State& state) const {
    const float altitude_m = state.location.alt * 0.01;

    return altitude_m >= min_allowed_alt_m.get() && altitude_m <= max_allowed_alt_m.get();
}

bool AP_GPS::AP_GPS_Validator::is_time_ok(const AP_GPS::GPS_State& state, uint64_t gps_time_us) const {
    return (gps_time_us - last_gps_time_us) >= (time_accuracy_ms * 1e3);
}

AP_GPS::AP_GPS_Validator::Action AP_GPS::AP_GPS_Validator::get_gps_failure_action() const {
    const auto action = action_on_failure.get();
    if (action > static_cast<int8_t>(Action::LAST) || action < static_cast<int8_t>(Action::FIRST)) {
        return Action::DO_NOT_INFROM;
    }
    return static_cast<Action>(action);
}

bool AP_GPS::AP_GPS_Validator::should_inform(Action action) {
    return action != Action::DO_NOT_INFROM;
}


#endif  // AP_GPS_ENABLED
