#include "ArduHumanoid.h"

/*
 * Init and run calls for stand mode
 * Maintains stable upright posture using EKF3 attitude feedback
 * with PD+G joint correction on hip and ankle joints
 */

bool ModeStand::init(bool ignore_checks)
{
    // Reset error integrals on mode entry
    _pitch_error_last = 0.0f;
    _roll_error_last = 0.0f;
    return true;
}

void ModeStand::run()
{
    const float dt = humanoid.scheduler.get_last_loop_time_s();

    // Read torso attitude from EKF3
    const float current_pitch = ahrs.get_pitch();
    const float current_roll  = ahrs.get_roll();

    // Compute errors relative to upright (0 degrees)
    const float pitch_error = 0.0f - current_pitch;
    const float roll_error  = 0.0f - current_roll;

    // Compute error rates
    const float pitch_error_rate = (pitch_error - _pitch_error_last) / dt;
    const float roll_error_rate  = (roll_error  - _roll_error_last)  / dt;

    // PD correction
    const float pitch_correction = 1.0f * pitch_error + 0.1f * pitch_error_rate;
    const float roll_correction  = 1.0f * roll_error  + 0.1f * roll_error_rate;

    // Apply corrections using available joint outputs
    // Hip pitch mapped to front/back movement
    // Ankle correction mapped to right/left balance
    humanoid.motors->front_out = pitch_correction;
    humanoid.motors->right_out = roll_correction;

    // Store last errors for derivative
    _pitch_error_last = pitch_error;
    _roll_error_last  = roll_error;
}