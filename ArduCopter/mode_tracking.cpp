#include "Copter.h"

#if MODE_TRACKING_ENABLED

/*
  TRACKING flight mode
  ====================
  Controls roll and pitch/throttle from external seeker errors via TRACKING_MESSAGE.

  errorx/errory arrive normalised in [-1, 1] and are converted to radians
  by GCS_MAVLink_Copter before being passed here.

  Roll     : errorx → lean angle (right-positive).
  Vertical : errory > 0 (target above) → nose-up pitch via pitch PID.
             errory < 0 (target below) → throttle reduction via same PID,
                         pitch held level so the copter descends in place.
  Yaw      : rate held at zero — copter holds current heading.
  Throttle : base = TRK_THROTTLE; below-target correction subtracts PID output
             scaled to [-lean_max, 0] throttle delta.
*/


bool ModeTracking::init(bool ignore_checks)
{
    _errorx_rad      = 0.0f;
    _errory_rad      = 0.0f;
    _prev_update_ms  = AP_HAL::millis();
    _lock_stable_ms  = AP_HAL::millis();
    _last_log_ms     = 0;

    g2.tracking_roll_pid.reset_I();
    g2.tracking_roll_pid.reset_filter();
    g2.tracking_pitch_pid.reset_I();
    g2.tracking_pitch_pid.reset_filter();

    gcs().send_text(MAV_SEVERITY_INFO, "Tracking: active");
    return true;
}

void ModeTracking::exit()
{
    g2.tracking_roll_pid.reset_I();
    g2.tracking_roll_pid.reset_filter();
    g2.tracking_pitch_pid.reset_I();
    g2.tracking_pitch_pid.reset_filter();
    gcs().send_text(MAV_SEVERITY_INFO, "Tracking: exit");
}

void ModeTracking::handle_tracking_error(float errorx_rad, float errory_rad)
{
    _errorx_rad = errorx_rad;
    _errory_rad = errory_rad;
}

void ModeTracking::run()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float dt = constrain_float((now_ms - _prev_update_ms) * 1e-3f, 0.001f, 0.5f);
    _prev_update_ms = now_ms;

    const float deadband_rad = radians(g2.tracking_deadband_deg.get());

    // Settle ramp — 0→1 over TRK_SETTLE_S from mode entry.
    const float settle_s = g2.tracking_settle_s.get();
    const float elapsed  = constrain_float((now_ms - _lock_stable_ms) * 1e-3f, 0.0f, settle_s);
    const float ramp     = (settle_s > 0.0f) ? (elapsed / settle_s) : 1.0f;

    const float ex_raw = fabsf(_errorx_rad) > deadband_rad ? _errorx_rad : 0.0f;
    const float ey_raw = fabsf(_errory_rad) > deadband_rad ? _errory_rad : 0.0f;

    const float lean_max_rad = attitude_control->lean_angle_max_rad();

    // ── Roll: errorx > 0 → target right → bank right ─────────────────────────
    float target_roll_rad = 0.0f;
    if (is_zero(ex_raw)) {
        g2.tracking_roll_pid.reset_I();
    } else {
        const float roll_cd = g2.tracking_roll_pid.update_all(
                                  degrees(ex_raw), 0.0f, dt) * ramp;
        target_roll_rad = constrain_float(
                              radians(roll_cd * 0.01f),
                              -lean_max_rad, lean_max_rad);
    }

    // ── Pitch / Throttle: vertical error drives attitude (above) or throttle (below) ──
    float target_pitch_rad = 0.0f;
    float throttle = constrain_float(g2.tracking_throttle.get() * ramp, 0.0f, 1.0f);

    if (is_zero(ey_raw)) {
        g2.tracking_pitch_pid.reset_I();
    } else {
        const float pitch_cd = g2.tracking_pitch_pid.update_all(
                                   degrees(ey_raw), 0.0f, dt) * ramp;
        if (ey_raw > 0.0f) {
            // target above → nose-up pitch (negative in copter convention)
            target_pitch_rad = constrain_float(
                                   -radians(pitch_cd * 0.01f),
                                   -lean_max_rad, lean_max_rad);
        } else {
            // target below → reduce throttle, keep pitch level
            // scale: cd→deg (×0.01) then deg→rad, divided by lean_max_rad
            const float throttle_delta = (pitch_cd * 0.01f) * DEG_TO_RAD / lean_max_rad;
            throttle = constrain_float(throttle + throttle_delta, 0.0f, 1.0f);
        }
    }

    // ── Motor spool ───────────────────────────────────────────────────────────
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0.0f, true, g.throttle_filt);
        return;
    default:
        break;
    }

    // ── Attitude + throttle ───────────────────────────────────────────────────
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        target_roll_rad, target_pitch_rad, 0.0f);
    attitude_control->set_throttle_out(throttle, true, g.throttle_filt);

    // ── Periodic log ──────────────────────────────────────────────────────────
    if (now_ms - _last_log_ms >= 1000U) {
        _last_log_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "TRK ex=%.2f ey=%.2f r=%.1f p=%.1f thr=%.2f",
                        (double)ex_raw, (double)ey_raw,
                        (double)degrees(target_roll_rad),
                        (double)degrees(target_pitch_rad),
                        (double)throttle);
    }
}

#endif  // MODE_TRACKING_ENABLED
