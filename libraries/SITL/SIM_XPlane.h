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
/*
  simulator connection for ardupilot version of Xplane
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_XPLANE_ENABLED

#include <AP_HAL/utility/Socket.h>   // generic SocketAPM (uses host sockets on SITL, lwIP on ChibiOS)
#include <AP_Filesystem/AP_Filesystem.h>

#include "SIM_Aircraft.h"
#include <AP_JSON/AP_JSON.h>

namespace SITL {

/*
  a Xplane simulator
 */
class XPlane : public Aircraft {
public:
    XPlane(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW XPlane(frame_str);
    }

private:

    bool receive_data(void);
    void send_dref(const char *name, float value);
    void request_drefs(void);
    void request_dref(const char *name, uint8_t code, uint32_t rate_hz);
    void send_drefs(const struct sitl_input &input);
    void handle_rref(const uint8_t *p, uint32_t len);
    void select_data(void);
    void deselect_code(uint8_t code);
    int8_t find_data_index(uint8_t id);

    // return true if at least X
    bool is_xplane12(void) const {
        return xplane_version / 10000 >= 12;
    }
    
    const char *xplane_ip = "127.0.0.1";
    char xplane_ip_buf[32] {};   // stores IP slice when frame_str has host:port
    uint16_t xplane_port = 49000;
    uint16_t bind_port = 49001;
    // udp socket, input and output
    SocketAPM socket_in{true};
    SocketAPM socket_out{true};

    uint64_t time_base_us;
    uint32_t last_data_time_ms;
    Vector3d position_zero;
    Vector3f accel_earth;
    // End of the settle window opened when X-Plane repositions ("teleports") the
    // aircraft. While inside it we feed a clean, level, stationary IMU frame so
    // the reposition transient never reaches the EKF. 0 = not settling.
    uint32_t teleport_settle_ms = 0;
    // Zulu time-of-day (seconds since 00:00 UTC) from X-Plane row 1; -1 = not yet
    // received. Seeds SITL start_time_UTC (year/date from build) so the simulated
    // GPS reports a sane UTC instead of underflowing.
    float xplane_zulu_sec = -1;
    // Local time-of-day (sec since 00:00 local) from X-Plane row 1 field 6; used
    // with zulu to derive the tz offset so the LOCAL date dref maps to the zulu
    // date. -1 = not yet received.
    float xplane_local_sec = -1;
    // 0-based day-of-year from sim/time/local_date_days (RREF). -1 = not yet
    // received → fall back to the build date's day-of-year.
    int xplane_doy = -1;
    bool time_synced = false;
    void seed_start_time_utc(void);
    bool connected = false;
    uint32_t xplane_frame_time;
    uint64_t seen_mask;
    uint32_t last_dref_ms;

    struct {
        uint32_t last_report_ms;
        uint32_t data_count;
        uint32_t frame_count;
    } report;

    enum class DRefType {
        ANGLE = 0,
        RANGE = 1,
        FIXED = 2,
    };

    struct DRef {
        struct DRef *next;
        char *name;
        DRefType type;
        uint8_t channel;
        float range;
        float fixed_value;
        bool  invert;       // negate ANGLE/RANGE value before sending
                            // (e.g. paired aileron from a single channel)
    };

    // list of DRefs;
    struct DRef *drefs;
    // round-robin cursor: send_drefs() walks the list one DRef per call
    // on PPP so the link's TX queue doesn't overflow. nullptr means "start
    // over from the head".
    struct DRef *next_dref_to_send;
    uint32_t dref_debug;

    enum class JoyType {
        AXIS = 0,
        BUTTON = 1,
    };

    // list of joystick inputs
    struct JoyInput {
        struct JoyInput *next;
        uint8_t axis;
        uint8_t channel;
        JoyType type;
        float input_min, input_max;
        uint32_t mask;
    };
    struct JoyInput *joyinputs;

    char *map_filename;
    struct stat map_st;

    bool load_dref_map(const char *map_json);
    void add_dref(const char *name, DRefType type, const AP_JSON::value &dref);
    void add_joyinput(const char *name, JoyType type, const AP_JSON::value &d);
    void handle_setting(const AP_JSON::value &d);

    void check_reload_dref(void);

    uint32_t xplane_version;
};


} // namespace SITL


#endif  // AP_SIM_XPLANE_ENABLED
