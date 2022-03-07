#include "mode.h"
#include "Plane.h"

void ModeStabilize::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = -1500;
    plane.pitchController.reset_I();
    plane.rollController.reset_I();
}

