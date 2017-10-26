#include "mode.h"
#include "Rover.h"

void ModeManual::update()
{
    // check for radio failsafe
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
    } else {
        // copy RC scaled inputs to outputs
        g2.motors.set_throttle(channel_throttle->get_control_in());
        g2.motors.set_steering(channel_steer->get_control_in());
        if (g.skid_steer_in) {
            // if in skid_steer_in, the radio already do the mixing
            g2.motors.use_skid_mixer(false);
        } else {
            g2.motors.use_skid_mixer(true);
        }
    }

    // mark us as in_reverse when using a negative throttle to stop AHRS getting off
    rover.set_reverse(is_negative(g2.motors.get_throttle()));
}
