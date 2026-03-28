#include "ArduHumanoid.h"

#include "RC_Channel_Humanoid.h"


// defining these two macros and including the RC_Channels_VarInfo header defines the parameter information common to all vehicle types
#define RC_CHANNELS_SUBCLASS RC_Channels_Humanoid
#define RC_CHANNEL_SUBCLASS RC_Channel_Humanoid

#include <RC_Channel/RC_Channels_VarInfo.h>

int8_t RC_Channels_Humanoid::flight_mode_channel_number() const
{
    return humanoid.g.flight_mode_chan.get();
}

void RC_Channel_Humanoid::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > humanoid.num_flight_modes) {
        // should not have been called
        return;
    }

    if (!humanoid.set_mode((Mode::Number)humanoid.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        // alert user to mode change failure
        if (humanoid.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return;
    }

    // play a tone
    // alert user to mode change (except if autopilot is just starting up)
    if (humanoid.ap.initialised) {
        AP_Notify::events.user_mode_change = 1;
    }
}

bool RC_Channels_Humanoid::in_rc_failsafe() const
{
    return humanoid.failsafe.radio;
}

bool RC_Channels_Humanoid::has_valid_input() const
{
    if (humanoid.failsafe.radio) {
        return false;
    }
    if (humanoid.failsafe.radio_counter != 0) {
        return false;
    }
    return RC_Channels::has_valid_input();
}

RC_Channel * RC_Channels_Humanoid::get_arming_channel(void) const
{
    return humanoid.channel_yaw;
}

// init_aux_switch_function - initialize aux functions
void RC_Channel_Humanoid::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch (ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::MANUAL:
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_function_change_mode - change mode based on an aux switch
// being moved
void RC_Channel_Humanoid::do_aux_function_change_mode(const Mode::Number mode,
        const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        const bool success = humanoid.set_mode(mode, ModeReason::AUX_FUNCTION);
        if (humanoid.ap.initialised) {
            if (success) {
                AP_Notify::events.user_mode_change = 1;
            } else {
                AP_Notify::events.user_mode_change_failed = 1;
            }
        }
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (humanoid.control_mode == mode) {
            rc().reset_mode_switch();
        }
    }
}

// do_aux_function - implement the function invoked by auxiliary switches
bool RC_Channel_Humanoid::do_aux_function(const AuxFuncTrigger &trigger)
{
    const AUX_FUNC &ch_option = trigger.func;
    const AuxSwitchPos &ch_flag = trigger.pos;

    switch (ch_option) {

    case AUX_FUNC::LOITER:
        do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
        break;

    case AUX_FUNC::MANUAL:
        do_aux_function_change_mode(Mode::Number::MANUAL, ch_flag);
        break;

    default:
        return RC_Channel::do_aux_function(trigger);
    }
    return true;
}
