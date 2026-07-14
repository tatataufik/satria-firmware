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

#include "AP_GPS_SITL.h"

#if AP_SIM_GPS_ENABLED

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

extern const AP_HAL::HAL& hal;

/*
  return GPS time of week in milliseconds
 */

// UTC midnight (seconds since 1970) of the firmware build date, from __DATE__
// ("Mmm dd yyyy").
static time_t sitl_build_date_midnight_utc(void)
{
    const char *d = __DATE__;
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    int mon = 0;
    for (int i=0; i<12; i++) {
        if (d[0]==months[i*3] && d[1]==months[i*3+1] && d[2]==months[i*3+2]) { mon = i; break; }
    }
    const int day  = atoi(d+4);
    const int year = atoi(d+7);
    auto is_leap = [](int y){ return (y%4==0 && y%100!=0) || y%400==0; };
    int64_t days = 0;
    for (int y=1970; y<year; y++) days += is_leap(y) ? 366 : 365;
    static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    for (int mo=0; mo<mon; mo++) days += mdays[mo] + ((mo==1 && is_leap(year)) ? 1 : 0);
    days += day - 1;
    return (time_t)(days * 86400LL);
}

/*
  get timeval using simulation time
 */
static void simulation_timeval(struct timeval *tv)
{
    const uint64_t now = AP_HAL::micros64();
    // Re-read start_time_UTC every call (no first_tv cache): on HIL hardware it is 0
    // at boot and only gets seeded later from X-Plane's zulu clock
    // (SIM_XPlane::seed_start_time_utc). Caching it would freeze the GPS at the
    // pre-seed (underflowed) time and never pick up the correction.
    time_t base = AP::sitl()->start_time_UTC;
    if (base == 0) {
        // Not yet seeded. Fall back to the build-date midnight so the first GPS fix
        // reports a sane ~build-date time instead of an underflowed far-future one.
        // It is always earlier than the real zulu-corrected time, so the later
        // correction moves the clock FORWARD -- which AP_RTC accepts (it refuses
        // backward jumps and would otherwise latch the bad value permanently).
        base = sitl_build_date_midnight_utc();
    }
    tv->tv_sec  = base + (time_t)(now / 1000000ULL);
    tv->tv_usec = now % 1000000ULL;
}
static void gps_time(uint16_t *time_week, uint32_t *time_week_ms)
{
    struct timeval tv;
    simulation_timeval(&tv);
    const uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - (GPS_LEAPSECONDS_MILLIS / 1000ULL);
    uint32_t epoch_seconds = tv.tv_sec - epoch;
    *time_week = epoch_seconds / AP_SEC_PER_WEEK;
    uint32_t t_ms = tv.tv_usec / 1000;
    // round time to nearest 200ms
    *time_week_ms = (epoch_seconds % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC + ((t_ms/200) * 200);
}

bool AP_GPS_SITL::read(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 200) {
        return false;
    }
    last_update_ms = now;

    auto *sitl = AP::sitl();

    double latitude =sitl->state.latitude;
    double longitude = sitl->state.longitude;
    float altitude = sitl->state.altitude;
    const double speedN = sitl->state.speedN;
    const double speedE = sitl->state.speedE;
    const double speedD = sitl->state.speedD;
    // const double yaw = sitl->state.yawDeg;

    uint16_t time_week;
    uint32_t time_week_ms;

    gps_time(&time_week, &time_week_ms);

    state.time_week = time_week;
    state.time_week_ms = time_week_ms;
    state.status = AP_GPS_FixType::FIX_3D;
    state.num_sats = 15;

    state.location = Location{
        int32_t(latitude*1e7),
        int32_t(longitude*1e7),
        int32_t(altitude*100),
        Location::AltFrame::ABSOLUTE
    };

    state.hdop = 100;
    state.vdop = 100;

    state.have_vertical_velocity = true;
    state.velocity.x = speedN;
    state.velocity.y = speedE;
    state.velocity.z = speedD;

    velocity_to_speed_course(state);

    state.have_speed_accuracy = true;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;
    state.have_vertical_velocity = true;

    // state.horizontal_accuracy = pkt.horizontal_pos_accuracy;
    // state.vertical_accuracy = pkt.vertical_pos_accuracy;
    // state.speed_accuracy = pkt.horizontal_vel_accuracy;

    state.last_gps_time_ms = now;

    return true;
}

#endif  // AP_SIM_GPS_ENABLED
