#include "mode.h"
#include "Plane.h"

void ModeStabilize::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
    plane.nav_pitch_rate_cds = 0; // this should only be non zero when doing a FW landing flare maneouvre
}

void ModeStabilize::run()
{
    plane.stabilize_roll();
    plane.stabilize_pitch();
    stabilize_stick_mixing_direct();
    plane.stabilize_yaw();
}
