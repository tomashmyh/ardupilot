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

#include <AP_gtest.h>

#include <AP_GPS/AP_GPS.h>

#include "gps_state_builder.h"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class TestGPSValidator : public AP_GPS::AP_GPS_Validator {
public:
    void set_now_ms(uint32_t t) { now_ms_ = t; }
protected:
    uint32_t now_ms() const override { return now_ms_; }
private:
    uint32_t now_ms_{0};
};

// Construct a "good" state sample.
// Note: last_gps_time_ms is used by speed gates; now_ms is used by time gate and debounce.
static AP_GPS::GPS_State make_state(uint8_t instance,
                                    uint32_t last_gps_time_ms,
                                    double lat_deg,
                                    double lon_deg,
                                    float alt_m,
                                    uint8_t sats)
{
    AP_GPS::GPS_State state = GPSStateBuilder::valid()
        .change_instance(instance)
        .change_last_gps_time_ms(last_gps_time_ms)
        .change_location(GPSStateBuilder::make_loc_deg_m(lat_deg, lon_deg, alt_m))
        .set_num_sats(sats)
        .build();

    return state;
}

static bool feed_state(TestGPSValidator& validator, uint32_t now_ms, const AP_GPS::GPS_State& state)
{
    validator.set_now_ms(now_ms);
    return validator.trust_gps(state);
}

static AP_GPS::GPS_State make_good(uint32_t t_ms)
{
    return make_state(/*instance*/0, /*last_gps_time_ms*/t_ms, 50.4501, 30.5234, 1900.0f, /*sats*/10);
}

// Drive the validator with a "good" sample at given time, returning trust result.
static bool feed_good(TestGPSValidator& validator, uint64_t now_ms, uint32_t last_gps_ms)
{
    return feed_state(validator, now_ms, make_good(now_ms));
}

// Bring validator to committed GOOD state (false -> true) by holding good for 5 seconds.
// Initial is_gps_good is false and becomes true only after debounce.
static void establish_good_committed(TestGPSValidator& validator, uint32_t t0_ms)
{
    // Start good at t0; should not yet flip to true.
    ASSERT_FALSE(feed_good(validator, t0_ms, t0_ms));
    // At t0+5000ms, debounce expires and commits to good.
    ASSERT_FALSE(feed_good(validator, t0_ms + 10, t0_ms + 10));
    ASSERT_TRUE(feed_good(validator, t0_ms + 10 + 5000, t0_ms + 10 + 5000));
}

TEST(AP_GPS_Validator, Disabled_AlwaysFalse)
{
    TestGPSValidator validator{};
    // enabled by default per param
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    validator.set_now_ms(0);
    auto bad = make_state(0, 0, 50.4501, 30.5234, 3000.0f, 0); // multiple failures
    ASSERT_FALSE(validator.trust_gps(bad));
}

TEST(AP_GPS_Validator, Startup_DebounceToTrue)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    // At t=0: still false (debounce not elapsed)
    ASSERT_FALSE(feed_good(validator, 0, 0));

    // At t=100ms: still false
    ASSERT_FALSE(feed_good(validator, 100, 100));
    ASSERT_FALSE(feed_good(validator, 5099, 5099));

    // At t=5000ms: becomes true
    ASSERT_TRUE(feed_good(validator, 5109, 5109));
}

TEST(AP_GPS_Validator, ImmediateFailure_TrueToFalse)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    // Commit to good
    establish_good_committed(validator, 0);

    // First detected failure => immediate false + single warning
    validator.set_now_ms(6000);
    auto bad = make_state(0, 6000, 50.4501, 30.5234, 100.0f, 0); // sats fail
    ASSERT_FALSE(validator.trust_gps(bad));

    // Subsequent failing samples
    validator.set_now_ms(7000);
    auto bad2 = bad;
    bad2.last_gps_time_ms = 7000;
    ASSERT_FALSE(validator.trust_gps(bad2));
}

TEST(AP_GPS_Validator, IsFirstDetectedReason_Priority_SATS_over_ALT)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // sats fail AND altitude invalid; must report sats (first failure)
    validator.set_now_ms(6000);
    auto bad = make_state(0, 6000, 50.4501, 30.5234, 3000.0f, 0);
    ASSERT_FALSE(validator.trust_gps(bad));
}

TEST(AP_GPS_Validator, IsFirstDetectedReason_Priority_HSPEED_before_ALT)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Provide a reference sample at t=6000
    validator.set_now_ms(6000);
    auto s1 = make_state(0, 6000, 50.4501, 30.5234, 1900.0f, 10);
    ASSERT_TRUE(validator.trust_gps(s1));

    // Next sample at t=7000, gps dt=1000ms, move 100m north => 100m/s > 30 => HSPEED failure.
    validator.set_now_ms(7000);
    auto s2 = make_state(0, 7000, 50.4501, 30.5234, 3000.0f, 10); // also altitude invalid
    s2.location = s1.location;
    s2.location.lat += GPSStateBuilder::dlat_1e7_from_north_m(100.0);
    s2.location.alt = static_cast<int32_t>(3000.0f * 100.0f); // keep ALT invalid to test priority
    ASSERT_FALSE(validator.trust_gps(s2));
}

TEST(AP_GPS_Validator, VerticalSpeedFailure_Immediate)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Reference sample
    validator.set_now_ms(6000);
    auto s1 = make_state(0, 6000, 50.4501, 30.5234, 1900.0f, 10);
    ASSERT_TRUE(validator.trust_gps(s1));

    // Next sample: dt=1s, +20m altitude => 20m/s > 15 => VSPEED failure
    validator.set_now_ms(7000);
    auto s2 = make_state(0, 7000, 50.4501, 30.5234, 120.0f, 10);
    ASSERT_FALSE(validator.trust_gps(s2));
}

TEST(AP_GPS_Validator, AltitudeFailure_Immediate)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 6000);

    validator.set_now_ms(6000);
    auto bad = make_state(0, 6000, 50.4501, 30.5234, 2500.0f, 10); // > 2000m
    ASSERT_FALSE(validator.trust_gps(bad));
}

TEST(AP_GPS_Validator, TimeFailure_Immediate)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Reference sample at t=10000
    ASSERT_TRUE(feed_good(validator, 10000, 10000));

    // Next call at t=10005 => 5ms < 10ms => TIME failure, immediate false
    validator.set_now_ms(10005);
    auto s2 = make_state(0, 10005, 50.4501, 30.5234, 1900.0f, 10);
    ASSERT_FALSE(validator.trust_gps(s2));
}

TEST(AP_GPS_Validator, RecoveryIsDebounced_FalseToTrueAfter5s)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    // Commit good then fail immediately (sats)
    establish_good_committed(validator, 0);

    validator.set_now_ms(6000);
    auto bad = make_state(0, 6000, 50.4501, 30.5234, 1900.0f, 0);
    ASSERT_FALSE(validator.trust_gps(bad));

    // Good again: start recovery timer at t=7000
    ASSERT_FALSE(feed_good(validator, 7000, 7000));    // still false

    // Not yet 5s of continuous good
    ASSERT_FALSE(feed_good(validator, 11999, 11999));

    // At t=12000 (>= 5s since 7000): commit to true; info once
    ASSERT_TRUE(feed_good(validator, 12010, 12010));

    // Further good samples: no additional info
    ASSERT_TRUE(feed_good(validator, 13000, 13000));
}

TEST(AP_GPS_Validator, RecoveryTimerResetsOnNewFailure)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Enter bad
    validator.set_now_ms(6000);
    auto bad = make_state(0, 6000, 50.4501, 30.5234, 100.0f, 0);
    ASSERT_FALSE(validator.trust_gps(bad));

    // Start recovery at t=7000
    ASSERT_FALSE(feed_good(validator, 7000, 7000));

    // New failure at t=8000 cancels recovery; no new warning (still bad)
    validator.set_now_ms(8000);
    auto bad2 = make_state(0, 8000, 50.4501, 30.5234, 2500.0f, 10); // altitude fail
    ASSERT_FALSE(validator.trust_gps(bad2));

    // Restart recovery at t=9000
    ASSERT_FALSE(feed_good(validator, 9000, 9000));
    ASSERT_FALSE(feed_good(validator, 13999, 13999)); // < 5s

    // At t=14000 => 5s after 9000 => recovers
    ASSERT_TRUE(feed_good(validator, 19010, 19010));
}

TEST(AP_GPS_Validator, DifferentFailuresWhileBad)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // First failure: sats -> warning once
    validator.set_now_ms(6000);
    auto bad_sats = make_state(0, 6000, 50.4501, 30.5234, 100.0f, 0);
    ASSERT_FALSE(validator.trust_gps(bad_sats));

    // Still bad, different failure reason now (altitude)
    validator.set_now_ms(7000);
    auto bad_alt = make_state(0, 7000, 50.4501, 30.5234, 2500.0f, 10);
    ASSERT_FALSE(validator.trust_gps(bad_alt));
}

TEST(AP_GPS_Validator, SATS_Failure_WhenOnlySatsBad)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Only SATS bad; keep everything else nominal.
    auto bad_sats = make_state(0, 6000, 50.4501, 30.5234, 100.0f, /*sats*/0);
    ASSERT_FALSE(feed_state(validator, 6000, bad_sats));
}

TEST(AP_GPS_Validator, HSPEED_Failure_WhenOnlyHorizontalSpeedBad)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Provide a reference good sample at t=6000 to establish last_state for speed computations.
    auto s1 = make_good(6000);
    ASSERT_TRUE(feed_state(validator, 6000, s1));

    // Next sample at t=7000: move 100m north in 1s -> 100m/s (HSPEED fail).
    auto s2 = make_good(7000);
    s2.location = s1.location;
    s2.location.lat += GPSStateBuilder::dlat_1e7_from_north_m(100.0); // horizontal motion only
    // Altitude unchanged => VSPEED passes; sats ok; time ok (dt=1000ms)
    ASSERT_FALSE(feed_state(validator, 7000, s2));
}

TEST(AP_GPS_Validator, VSPEED_Failure_WhenOnlyVerticalSpeedBad)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Reference good sample at t=6000
    auto s1 = make_good(6000);
    ASSERT_TRUE(feed_state(validator, 6000, s1));

    // Next at t=7000: +20m in 1s -> 20m/s (VSPEED fail if max is 15 as implied by your tests)
    auto s2 = make_good(7000);
    s2.location = s1.location;                         // no horizontal motion => HSPEED passes
    s2.location.alt = static_cast<int32_t>(120.0f * 100.0f); // 120m (cm)
    ASSERT_FALSE(feed_state(validator, 7000, s2));
}

TEST(AP_GPS_Validator, ALT_Failure_WhenOnlyAltitudeBad)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 6000);

    // Choose a time that trivially passes TIME gate (>=10ms since last)
    auto bad_alt = make_state(0, 6000, 50.4501, 30.5234, 2500.0f, /*sats*/10);
    ASSERT_FALSE(feed_state(validator, 6000, bad_alt));
}

TEST(AP_GPS_Validator, TIME_Failure_WhenOnlyTimeBad)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Reference good sample at t=10000
    ASSERT_TRUE(feed_good(validator, 10000, 10000));

    // Next at t=10005: dt(now)=5ms < 10ms => TIME fail.
    // Keep location and alt stable so HSPEED/VSPEED pass despite small dt.
    auto s2 = make_state(0, 10005, 50.4501, 30.5234, 1900.0f, 10);
    ASSERT_FALSE(feed_state(validator, 10005, s2));
}

TEST(AP_GPS_Validator, NONE_WhenAllChecksPass_AndAlreadyCommittedGood)
{
    TestGPSValidator validator{};
    validator.apply_enable_state(true);
    validator.change_action_on_failure(AP_GPS::AP_GPS_Validator::Action::ONLY_DISABLE_GPS_USE);

    establish_good_committed(validator, 0);

    // Once committed good, another good sample should remain trusted.
    ASSERT_TRUE(feed_good(validator, 6000, 6000));
}

AP_GTEST_MAIN()
