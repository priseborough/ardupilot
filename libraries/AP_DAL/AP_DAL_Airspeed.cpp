#include "AP_DAL_Airspeed.h"
#include "AP_DAL.h"

#include <AP_Logger/AP_Logger.h>

AP_DAL_Airspeed::AP_DAL_Airspeed()
{
#if AP_AIRSPEED_ENABLED
    for (uint8_t i=0; i<ARRAY_SIZE(_RASI); i++) {
        _RASI[i].instance = i;
    }
#endif
}

void AP_DAL_Airspeed::start_frame()
{
#if AP_AIRSPEED_ENABLED
    const auto *airspeed = AP::windvane();
    if (airspeed == nullptr) {
        return;
    }

    const log_RASH old = _RASH;
    _RASH.num_sensors = 1;
    _RASH.primary = 0;
    WRITE_REPLAY_BLOCK_IFCHANGED(RASH, _RASH, old);

    for (uint8_t i=0; i<ARRAY_SIZE(_RASI); i++) {
        log_RASI &RASI = _RASI[i];
        log_RASI old_RASI = RASI;
        RASI.last_update_ms = airspeed->last_update_ms();
        RASI.healthy = true;
        RASI.use = true;
        RASI.airspeed = airspeed->get_apparent_wind_speed_raw();
        RASI.direction = degrees(airspeed->get_apparent_wind_direction_rad_raw());
        WRITE_REPLAY_BLOCK_IFCHANGED(RASI, RASI, old_RASI);
    }
#endif
}
