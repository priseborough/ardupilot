#include "mode.h"
#include "Plane.h"

bool ModeFBWB::_enter()
{
    float posD;
    if (!ahrs.get_relative_position_D_origin(posD)) {
        // can't fly in this mode if no vertical position is available
        return false;
    }
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeFBWB::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    plane.update_fbwb_speed_height();

}

