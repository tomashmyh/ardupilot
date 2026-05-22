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
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_GPS::AP_GPS_Validator, is_enabled, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: ACTION
    // @DisplayName: Action on GPS validation failure
    // @Description: Defines an action involved when GPS identified as a bad
    // @Values: 0:OnlyInform,1:OnlyDisableGPSUse,2:InformAndDisableGPSUse
    // @User: Advanced
    AP_GROUPINFO("ACTION", 2, AP_GPS::AP_GPS_Validator, action_on_failure, static_cast<int8_t>(AP_GPS_Validator::Action::INFORM_AND_DISABLE_GPS_USE)),

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
    AP_GROUPINFO("V_H_MAX", 4, AP_GPS::AP_GPS_Validator, max_horizontal_speed_mps, 45),

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

    // @Param: MIN_DT_MS
    // @DisplayName: Minimum GPS update interval
    // @Description: Minimum wall-clock time in milliseconds that must elapse between consecutive GPS samples. Samples arriving faster than this are rejected as invalid.
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("MIN_DT", 8, AP_GPS::AP_GPS_Validator, min_dt_ms, 10),

    // @Param: LAT_MIN
    // @DisplayName: Min valid latitude
    // @Description: Defines a minimum valid latitude region
    // @Units: degree
    // @User: Advanced
    AP_GROUPINFO("LAT_MIN", 9, AP_GPS::AP_GPS_Validator, min_valid_lat, 45.0f),

    // @Param: LON_MIN
    // @DisplayName: Min valid longitude
    // @Description: Defines a minimum valid longitude region
    // @Units: degree
    // @User: Advanced
    AP_GROUPINFO("LON_MIN", 10, AP_GPS::AP_GPS_Validator, min_valid_lon, 27.0f),

    // @Param: LAT_MAX
    // @DisplayName: Max valid latitude
    // @Description: Defines a maximum valid latitude region
    // @Units: degree
    // @User: Advanced
    AP_GROUPINFO("LAT_MAX", 11, AP_GPS::AP_GPS_Validator, max_valid_lat, 53.0f),

    // @Param: LON_MAX
    // @DisplayName: Max valid longitude
    // @Description: Defines a maximum valid longitude region
    // @Units: degree
    // @User: Advanced
    AP_GROUPINFO("LON_MAX", 12, AP_GPS::AP_GPS_Validator, max_valid_lon, 45.0f),

    // @Param: INST
    // @DisplayName: GPS instance to validate
    // @Description: Defines a GPS instance to run the gps validation on
    // @Values: 0:First,1:Second,2:Primary
    // @User: Advanced
    AP_GROUPINFO("INST", 13, AP_GPS::AP_GPS_Validator, gps_instance_to_validate, static_cast<int8_t>(AP_GPS_Validator::GpsInstance::FIRST)),

    // @Param: TIME_TOL
    // @DisplayName: GPS time regression tolerance
    // @Description: Maximum allowed backwards step in GPS time-of-week (time_week_ms) between consecutive accepted samples, in milliseconds. Zero means strictly monotonic. Increase to 500-1000 for ublox receivers, which can produce small iTOW regressions during clock correction or re-acquisition. Does not affect the GPS week number check, which is always strict.
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("TIME_T", 14, AP_GPS::AP_GPS_Validator, gps_time_tolerance_ms, 500),

    // @Param: H_ACC_MAX
    // @DisplayName: Maximum horizontal accuracy
    // @Description: Maximum allowed horizontal position accuracy in meters. Samples with reported accuracy worse than this threshold are rejected. Set 0 to disable. Requires the GPS driver to report horizontal accuracy (have_horizontal_accuracy).
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("H_A_MAX", 15, AP_GPS::AP_GPS_Validator, max_h_accuracy_m, 5.0f),

    // @Param: V_ACC_MAX
    // @DisplayName: Maximum vertical accuracy
    // @Description: Maximum allowed vertical position accuracy in meters. Samples with reported accuracy worse than this threshold are rejected. Set 0 to disable. Requires the GPS driver to report vertical accuracy (have_vertical_accuracy).
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("V_A_MAX", 16, AP_GPS::AP_GPS_Validator, max_v_accuracy_m, 8.0f),

    // @Param: UND_MIN
    // @DisplayName: Minimum geoid undulation
    // @Description: Minimum allowed WGS84 geoid undulation (ellipsoid height minus MSL height) in meters. Samples outside this range are rejected. Requires the GPS driver to report undulation (have_undulation). Global range is approximately -120m to +90m.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("UND_MIN", 17, AP_GPS::AP_GPS_Validator, min_undulation_m, -120.0f),

    // @Param: UND_MAX
    // @DisplayName: Maximum geoid undulation
    // @Description: Maximum allowed WGS84 geoid undulation (ellipsoid height minus MSL height) in meters. Samples outside this range are rejected. Requires the GPS driver to report undulation (have_undulation). Global range is approximately -120m to +90m.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("UND_MAX", 18, AP_GPS::AP_GPS_Validator, max_undulation_m, 100.0f),

    // @Param: SAT_JUMP
    // @DisplayName: Maximum satellite count jump
    // @Description: Maximum allowed single-step increase in reported satellite count while already tracking. A real receiver acquires satellites gradually; a sudden large upward jump (e.g. 6 to 20) indicates a spoofing attack injecting many fake signals simultaneously. Only upward jumps are checked - drops are normal signal loss already covered by SAT_N. Set 0 to disable.
    // @User: Advanced
    AP_GROUPINFO("SAT_JMP", 19, AP_GPS::AP_GPS_Validator, max_sat_jump, 0),

    // @Param: LT_VH_MAX
    // @DisplayName: Maximum long-term horizontal speed
    // @Description: Maximum allowed average horizontal speed over the long-term anchor window (2s). Complements V_H_MAX: while V_H_MAX catches instantaneous position jumps, this catches sustained gradual drift that stays under the per-sample threshold. Set to no less than the vehicle's actual maximum horizontal speed.
    // @Units: mps
    // @User: Advanced
    AP_GROUPINFO("LT_VH", 20, AP_GPS::AP_GPS_Validator, max_lt_horizontal_speed_mps, 30),

    // @Param: LT_VV_MAX
    // @DisplayName: Maximum long-term vertical speed
    // @Description: Maximum allowed average vertical speed over the long-term anchor window (2s). Complements V_V_MAX: while V_V_MAX catches instantaneous altitude jumps, this catches sustained vertical drift that stays under the per-sample threshold. Set to no less than the vehicle's actual maximum vertical speed.
    // @Units: mps
    // @User: Advanced
    AP_GROUPINFO("LT_VV", 21, AP_GPS::AP_GPS_Validator, max_lt_vertical_speed_mps, 15),

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

int8_t AP_GPS::AP_GPS_Validator::get_instance_number(int8_t primary_instance) const {
    const auto desired_instance = gps_instance_to_validate.get();
    if (desired_instance == static_cast<int8_t>(GpsInstance::PRIMARY)) {
        return primary_instance;
    }
    if ((desired_instance < 0) || (desired_instance >= GPS_MAX_RECEIVERS)) {
        return 0;
    }

    return desired_instance;
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

    if (!candidate_good) {
        // become bad immediately
        if (is_gps_good) {
            // State change true -> false: send exactly one message (first detected failure)
            if (inform) {
                send_failure_text(fail, state.instance + 1);
            }
            is_gps_good = false;
            // Invalidate the anchor: the position is no longer trusted, so any
            // anchor-relative speed computed during recovery would be meaningless.
            anchor_state_valid = false;
        }
        // Cancel any pending recovery timer
        pending_good_valid = false;

    } else {
        // candidate_good == true

        if (is_gps_good) {
            // Already trusted: advance the baseline with each accepted sample so
            // speed checks track the vehicle's real position incrementally.
            last_state = state;
            last_state_valid = true;
            pending_good_valid = false;

            // Advance the long-term anchor every ANCHOR_INTERVAL_S seconds.
            // Between updates the anchor is fixed, so the long-term speed check
            // measures average displacement over a growing window (0..ANCHOR_INTERVAL_S).
            if (!anchor_state_valid ||
                (now - anchor_last_update_ms) >= (ANCHOR_INTERVAL_S * 1000U)) {
                anchor_state = state;
                anchor_state_valid = true;
                anchor_last_update_ms = now;
            }

        } else {
            // Recovering (debounce in progress): keep the baseline frozen at the
            // last pre-outage accepted position.

            // A gap larger than DEBOUNCE_MAX_GAP_MS between consecutive calls
            // means GPS stopped sending during recovery. Reset the debounce so
            // the GPS must sustain a full uninterrupted window to be trusted.
            // last_gps_time_ms still holds the timestamp of the previous call.
            if (pending_good_valid &&
                (last_gps_time_ms != UINT32_MAX) &&
                ((now - last_gps_time_ms) > DEBOUNCE_MAX_GAP_MS)) {
                pending_good_valid = false;
            }

            if (!pending_good_valid) {
                pending_good_valid = true;
                pending_good_since_ms = now;
            }

            const uint32_t elapsed_ms = now - pending_good_since_ms;
            if (elapsed_ms >= (CHANGE_STATE_DELAY_S * 1000U)) {
                // Debounce complete: anchor baseline to the sample that commits.
                last_state = state;
                last_state_valid = true;
                is_gps_good = true;
                pending_good_valid = false;

                // Start a fresh long-term anchor from the recovery point.
                anchor_state = state;
                anchor_state_valid = true;
                anchor_last_update_ms = now;

                if (inform) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %d: good", state.instance + 1);
                }
            }
        }
    }

    // Always advance the wall-clock stamp so the TIME gate can bootstrap and
    // correctly measure update rate on the next call, regardless of whether
    // this sample was accepted.
    last_gps_time_ms = now;

    // only ONLY_INFORM gates the return value
    return (action != Action::ONLY_INFORM) ? is_gps_good : true;
}

bool AP_GPS::AP_GPS_Validator::is_satellites_ok(const AP_GPS::GPS_State& state) const {
    return state.num_sats >= min_sat_count.get();
}

bool AP_GPS::AP_GPS_Validator::is_sat_count_ok(const AP_GPS::GPS_State& state) const {
    if (!last_state_valid || !is_gps_good) {
        return true;  // no baseline, or recovering - jump is expected, skip check
    }
    const int8_t limit = max_sat_jump.get();
    if (limit <= 0) {
        return true;  // disabled
    }
    // Only upward jumps are suspicious: a spoofing attack typically injects many
    // satellites simultaneously. A downward change is normal signal loss and is
    // already covered by the minimum satellite count check.
    const int16_t delta = static_cast<int16_t>(state.num_sats) -
                          static_cast<int16_t>(last_state.num_sats);
    return delta <= static_cast<int16_t>(limit);
}

bool AP_GPS::AP_GPS_Validator::is_horizontal_speed_ok(const AP_GPS::GPS_State& state, uint32_t now_ms) const {
    if (!last_state_valid) {
        return true;  // no baseline yet, skip check
    }
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
    if (!last_state_valid) {
        return true;  // no baseline yet, skip check
    }

    const int32_t dt_ms = int32_t(state.last_gps_time_ms - last_state.last_gps_time_ms);
    if (dt_ms <= 0) {
        return true;
    }
    const float time_diff_s = dt_ms * 0.001f;

    ftype altitude_diff_m = 0.0;
    if (!state.location.get_alt_distance(last_state.location, altitude_diff_m)) {
        return false;
    }

    const float vertical_speed_mps = fabsf(static_cast<float>(altitude_diff_m)) / time_diff_s;

    return vertical_speed_mps <= max_vertical_speed_mps;
}

bool AP_GPS::AP_GPS_Validator::is_long_term_horizontal_speed_ok(const AP_GPS::GPS_State& state) const {
    if (!anchor_state_valid) {
        return true;
    }
    const int32_t dt_ms = int32_t(state.last_gps_time_ms - anchor_state.last_gps_time_ms);
    if (dt_ms <= 0) {
        return true;
    }
    const float time_diff_s = dt_ms * 0.001f;
    const float dist_m = static_cast<float>(state.location.get_distance(anchor_state.location));
    return fabsf(dist_m) / time_diff_s <= max_lt_horizontal_speed_mps;
}

bool AP_GPS::AP_GPS_Validator::is_long_term_vertical_speed_ok(const AP_GPS::GPS_State& state) const {
    if (!anchor_state_valid) {
        return true;
    }
    const int32_t dt_ms = int32_t(state.last_gps_time_ms - anchor_state.last_gps_time_ms);
    if (dt_ms <= 0) {
        return true;
    }
    const float time_diff_s = dt_ms * 0.001f;
    ftype alt_diff_m = 0.0;
    if (!state.location.get_alt_distance(anchor_state.location, alt_diff_m)) {
        return false;
    }
    return fabsf(static_cast<float>(alt_diff_m)) / time_diff_s <= max_lt_vertical_speed_mps;
}

bool AP_GPS::AP_GPS_Validator::is_altitude_ok(const AP_GPS::GPS_State& state) const {
    const float altitude_m = state.location.alt * 0.01f;

    return altitude_m >= min_allowed_alt_m.get() && altitude_m <= max_allowed_alt_m.get();
}

bool AP_GPS::AP_GPS_Validator::is_position_ok(const AP_GPS::GPS_State& state) const {
    const float lat = state.location.lat * 1e-7;
    const float lon = state.location.lng * 1e-7;
    const bool is_lat_ok = lat > min_valid_lat && lat < max_valid_lat;
    const bool is_lon_ok = lon > min_valid_lon && lon < max_valid_lon;

    return is_lat_ok && is_lon_ok;
}

bool AP_GPS::AP_GPS_Validator::is_time_ok(const AP_GPS::GPS_State& state, uint32_t now_ms) const {
    if (last_gps_time_ms == UINT32_MAX) {
        return false; // no prior sample yet
    }
    // Reject samples that arrive faster than the configured minimum interval.
    // This is a wall-clock rate limiter, not a GPS time accuracy check.
    const uint32_t dt_ms = now_ms - last_gps_time_ms;
    return dt_ms >= static_cast<uint32_t>(min_dt_ms.get());
}

bool AP_GPS::AP_GPS_Validator::is_gps_time_ok(const AP_GPS::GPS_State& state) const {
    // time_week == 0 means the GPS has no satellite time lock - not a spoofing
    // indicator, so skip the check entirely rather than failing.
    if (state.time_week == 0) {
        return true;
    }
    // No accepted baseline yet; nothing to compare against.
    if (!last_state_valid || last_state.time_week == 0) {
        return true;
    }
    // GPS week number must never decrease - week rollover always increments it.
    // A decreasing week is an unambiguous spoofing/replay indicator regardless
    // of the GPS driver in use.
    if (state.time_week < last_state.time_week) {
        return false;
    }
    // Within the same week, time_week_ms (iTOW) must not regress beyond the
    // configured tolerance.
    if (state.time_week == last_state.time_week) {
        const int32_t delta_ms = static_cast<int32_t>(state.time_week_ms) -
                                 static_cast<int32_t>(last_state.time_week_ms);
        if (delta_ms < -static_cast<int32_t>(gps_time_tolerance_ms.get())) {
            return false;
        }
    }
    return true;
}

bool AP_GPS::AP_GPS_Validator::is_horizontal_accuracy_ok(const AP_GPS::GPS_State& state) const {
    if (!state.have_horizontal_accuracy) {
        return true;  // driver does not report accuracy, skip check
    }
    const float threshold = max_h_accuracy_m.get();
    if (threshold <= 0.0f) {
        return true;  // disabled
    }
    return state.horizontal_accuracy <= threshold;
}

bool AP_GPS::AP_GPS_Validator::is_vertical_accuracy_ok(const AP_GPS::GPS_State& state) const {
    if (!state.have_vertical_accuracy) {
        return true;  // driver does not report accuracy, skip check
    }
    const float threshold = max_v_accuracy_m.get();
    if (threshold <= 0.0f) {
        return true;  // disabled
    }
    return state.vertical_accuracy <= threshold;
}

bool AP_GPS::AP_GPS_Validator::is_undulation_ok(const AP_GPS::GPS_State& state) const {
    if (!state.have_undulation) {
        return true;  // driver does not report undulation, skip check
    }
    return state.undulation >= min_undulation_m.get() &&
           state.undulation <= max_undulation_m.get();
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
    if (!is_sat_count_ok(state)) {
        return FailureReason::SAT_JUMP;
    }
    if (!is_time_ok(state, now_ms)) {
        return FailureReason::TIME;
    }
    if (!is_gps_time_ok(state)) {
        return FailureReason::GPS_TIME;
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
    if (!is_position_ok(state)) {
        return FailureReason::POS;
    }
    if (!is_horizontal_accuracy_ok(state)) {
        return FailureReason::H_ACCURACY;
    }
    if (!is_vertical_accuracy_ok(state)) {
        return FailureReason::V_ACCURACY;
    }
    if (!is_undulation_ok(state)) {
        return FailureReason::UND;
    }
    if (!is_long_term_horizontal_speed_ok(state)) {
        return FailureReason::LONG_HSPEED;
    }
    if (!is_long_term_vertical_speed_ok(state)) {
        return FailureReason::LONG_VSPEED;
    }
    return FailureReason::NONE;
}

void AP_GPS::AP_GPS_Validator::send_failure_text(AP_GPS::AP_GPS_Validator::FailureReason reason, uint8_t gps_instance_plus1)
{
    switch (reason) {
    case AP_GPS::AP_GPS_Validator::FailureReason::SATS:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: bad sats", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::SAT_JUMP:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: sat count jump", gps_instance_plus1);
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
    case AP_GPS::AP_GPS_Validator::FailureReason::POS:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: bad pos", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::GPS_TIME:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: gps time regressed", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::H_ACCURACY:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: h accuracy too low", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::V_ACCURACY:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: v accuracy too low", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::UND:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: bad undulation", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::LONG_HSPEED:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: sustained hspeed", gps_instance_plus1);
        break;
    case AP_GPS::AP_GPS_Validator::FailureReason::LONG_VSPEED:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %d: sustained vspeed", gps_instance_plus1);
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
