#include "mode.h"
#include "Plane.h"

bool ModeAutoTune::_enter()
{
    plane.autotune_start();
    plane.nav_pitch_rate_cds = 0; // this should only be non zero when doing a FW landing flare maneouvre
    return true;
}


void ModeAutoTune::update()
{
    plane.mode_fbwa.update();
}

