#pragma once

#include <RC_Channel/RC_Channel.h>
#include "Joints.h"
#include "mode.h" //this includes Humanoid.h which includes Joints.h

class RC_Channel_Humanoid : public RC_Channel
{

public:

protected:

    void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    bool do_aux_function(const AuxFuncTrigger &trigger) override;

private:

    void do_aux_function_change_mode(const Mode::Number mode,
                                     const AuxSwitchPos ch_flag);
    void do_aux_function_change_air_mode(const AuxSwitchPos ch_flag);

    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

};

class RC_Channels_Humanoid : public RC_Channels
{
public:

    bool has_valid_input() const override;
    bool in_rc_failsafe() const override;

    RC_Channel *get_arming_channel(void) const override;

    RC_Channel_Humanoid obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Humanoid *channel(const uint8_t chan) override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    int8_t flight_mode_channel_number() const override;

};
