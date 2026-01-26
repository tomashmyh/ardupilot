/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_GPS/AP_GPS.h>

class GPSStateBuilder {
public:
    // Factory: create a baseline-valid state for tests.
    static GPSStateBuilder valid()  {
        AP_GPS::GPS_State state{};

        // Baseline "valid enough" defaults for tests
        state.instance = 0;
        state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
        state.time_week_ms = 0;
        state.time_week = 0;
        state.location = make_default_location();
        state.ground_speed = 0.0f;
        state.ground_course = 0.0f;

        state.gps_yaw = 0.0f;
        state.gps_yaw_time_ms = 0;
        state.gps_yaw_configured = false;

        state.hdop = 200;  // 2.00
        state.vdop = 200;  // 2.00
        state.num_sats = 100;

        state.velocity = make_default_velocity();
        state.speed_accuracy = 0.0f;
        state.horizontal_accuracy = 0.0f;
        state.vertical_accuracy = 0.0f;
        state.gps_yaw_accuracy = 0.0f;

        state.have_vertical_velocity = false;
        state.have_horizontal_velocity = false;
        state.have_speed_accuracy = false;
        state.have_horizontal_accuracy = false;
        state.have_vertical_accuracy = false;
        state.have_gps_yaw = false;
        state.have_gps_yaw_accuracy = false;

        state.undulation = 0.0f;
        state.have_undulation = false;

        state.last_gps_time_ms = 0;
        state.announced_detection = false;
        state.last_corrected_gps_time_us = 0;
        state.corrected_timestamp_updated = false;
        state.lagged_sample_count = 0;

        // RTK fields
        state.rtk_time_week_ms = 0;
        state.rtk_week_number = 0;
        state.rtk_age_ms = 0;
        state.rtk_num_sats = 0;
        state.rtk_baseline_coords_type = 1;
        state.rtk_baseline_x_mm = 0;
        state.rtk_baseline_y_mm = 0;
        state.rtk_baseline_z_mm = 0;
        state.rtk_accuracy = 0;
        state.rtk_iar_num_hypotheses = 0;

        // UBX relpos
        state.relPosHeading = 0.0f;
        state.relPosLength = 0.0f;
        state.relPosD = 0.0f;
        state.accHeading = 0.0f;
        state.relposheading_ts = 0;

        return GPSStateBuilder{state};
    }

    GPSStateBuilder& change_instance(uint8_t instance) {
        state.instance = instance;
        return *this;
    }

    GPSStateBuilder& change_fix(AP_GPS::GPS_Status status) {
        state.status = status;
        return *this;
    }

    GPSStateBuilder& change_time(uint16_t week, uint32_t week_ms) {
        state.time_week = week;
        state.time_week_ms = week_ms;
        return *this;
    }

    GPSStateBuilder& change_location(const Location& loc) {
        state.location = loc;
        return *this;
    }

    GPSStateBuilder& change_altitude(float alt_m) {
        state.location.alt = alt_m;
        return *this;
    }

    GPSStateBuilder& change_ground_speed(float mps) {
        state.ground_speed = mps;
        return *this;
    }

    GPSStateBuilder& change_ground_course(float deg_0_360) {
        state.ground_course = deg_0_360;
        return *this;
    }

    GPSStateBuilder& change_velocity_ned(const Vector3f& v_ned) {
        state.velocity = v_ned;
        state.have_horizontal_velocity = true;
        state.have_vertical_velocity = true;
        return *this;
    }

    GPSStateBuilder& change_last_gps_time_ms(uint32_t ms) {
        state.last_gps_time_ms = ms;
        return *this;
    }

    GPSStateBuilder& change_altitude_m(float m) {
        state.location.alt = int32_t(m * 100.0f);
        return *this;
    }

    GPSStateBuilder& change_latlon_deg(double lat, double lon) {
        state.location.lat = lat * 1e7;
        state.location.lng = lon * 1e7;
        return *this;
    }

    GPSStateBuilder& enable_gps_yaw(float yaw_deg, uint32_t ts_ms) {
        state.gps_yaw = yaw_deg;
        state.gps_yaw_time_ms = ts_ms;
        state.gps_yaw_configured = true;
        state.have_gps_yaw = true;
        return *this;
    }

    GPSStateBuilder& disable_gps_yaw() {
        state.gps_yaw_configured = false;
        state.have_gps_yaw = false;
        return *this;
    }

    GPSStateBuilder& set_hdop(uint16_t hdop_x100) {
        state.hdop = hdop_x100;
        return *this;
    }

    GPSStateBuilder& set_vdop(uint16_t vdop_x100) {
        state.vdop = vdop_x100;
        return *this;
    }

    GPSStateBuilder& set_num_sats(uint8_t n) {
        state.num_sats = n;
        return *this;
    }

    GPSStateBuilder& set_horizontal_accuracy(float m) {
        state.horizontal_accuracy = m;
        state.have_horizontal_accuracy = true;
        return *this;
    }

    GPSStateBuilder& set_vertical_accuracy(float m) {
        state.vertical_accuracy = m;
        state.have_vertical_accuracy = true;
        return *this;
    }

    GPSStateBuilder& set_speed_accuracy(float mps) {
        state.speed_accuracy = mps;
        state.have_speed_accuracy = true;
        return *this;
    }

    GPSStateBuilder& set_have_horizontal_accuracy(bool v) {
        state.have_horizontal_accuracy = v;
        return *this;
    }

    GPSStateBuilder& set_have_vertical_accuracy(bool v) {
        state.have_vertical_accuracy = v;
        return *this;
    }

    GPSStateBuilder& set_have_speed_accuracy(bool v) {
        state.have_speed_accuracy = v;
        return *this;
    }

    GPSStateBuilder& set_have_horizontal_velocity(bool v) {
        state.have_horizontal_velocity = v;
        return *this;
    }

    GPSStateBuilder& set_have_vertical_velocity(bool v) {
        state.have_vertical_velocity = v;
        return *this;
    }

    GPSStateBuilder& set_rtk_baseline_ned_mm(int32_t n_mm, int32_t e_mm, int32_t d_mm) {
        state.rtk_baseline_coords_type = 1; // NED
        state.rtk_baseline_x_mm = n_mm;
        state.rtk_baseline_y_mm = e_mm;
        state.rtk_baseline_z_mm = d_mm;
        return *this;
    }

    GPSStateBuilder& set_rtk_age_ms(uint32_t age_ms) {
        state.rtk_age_ms = age_ms;
        return *this;
    }

    GPSStateBuilder& set_rtk_num_sats(uint8_t n) {
        state.rtk_num_sats = n;
        return *this;
    }

    GPSStateBuilder& set_relpos_heading(float heading_deg, float acc_heading_deg, uint32_t ts) {
        state.relPosHeading = heading_deg;
        state.accHeading = acc_heading_deg;
        state.relposheading_ts = ts;
        state.gps_yaw = heading_deg;
        state.gps_yaw_accuracy = acc_heading_deg;
        state.have_gps_yaw = true;
        state.have_gps_yaw_accuracy = true;
        return *this;
    }

    AP_GPS::GPS_State build() const {
        return state;
    }

    const AP_GPS::GPS_State& get() const {
        return state;
    }

    static Location make_default_location() {
        return Location(50.37896, 32.56894, 900, Location::AltFrame::ABSOLUTE);
    }

    static Vector3f make_default_velocity() {
        // v.x = 0; v.y = 0; v.z = 0; // NED: (N,E,D)
        return {};
    }

    static Location make_loc_deg_m(double lat_deg, double lon_deg, float alt_m)
    {
        Location l{};
        l.lat = static_cast<int32_t>(llround(lat_deg * 1e7));
        l.lng = static_cast<int32_t>(llround(lon_deg * 1e7));
        l.alt = static_cast<int32_t>(llround(alt_m * 100.0f)); // cm
        return l;
    }

    // Convert north meters to delta-lat in 1e7-deg units.
    // (Ignoring ellipsoid, but for test deltas like 100m this is robust.)
    static int32_t dlat_1e7_from_north_m(double north_m)
    {
        constexpr double meters_per_deg_lat = 111319.49079327357; // WGS84-ish
        const double deg = north_m / meters_per_deg_lat;
        return static_cast<int32_t>(llround(deg * 1e7));
    }

private:
    AP_GPS::GPS_State state{};

    // initialize s_ to a consistent baseline.
    explicit GPSStateBuilder(AP_GPS::GPS_State baseline) : state(std::move(baseline)) {}
};