/*
 * AP_NPS.cpp
 *
 * Naval Postgraduate School custom module
 *
 */
#include <AP_NPS.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NPS::var_info[] PROGMEM = {
    // @Param: KILL_THR
    // @DisplayName: Force kill throttle
    // @Description: Can be set to 1 in flight to force throttle to 0 in an emergency.
    // @User: Advanced
    AP_GROUPINFO("KILL_THR", 0, AP_NPS, _kill_throttle, 0),

    AP_GROUPEND
};

AP_NPS::AP_NPS() {
    AP_Param::setup_object_defaults(this, var_info);
    _kill_throttle = 0;
    _current_fs_state = NO_FS;
}

AP_Int8 AP_NPS::get_kill_throttle() {
    //always kill throttle in GPS_LONG_FS
    if (_current_fs_state == GPS_LONG_FS) {
        AP_Int8 retVal;
        retVal.set(1);
        return retVal;
    }

    return _kill_throttle;
}

void AP_NPS::set_kill_throttle(AP_Int8 kt) {
    _kill_throttle = kt;
}

// check for failsafe conditions IN PRIORITY ORDER
void AP_NPS::check(NPS_FlightMode mode, 
                   AP_SpdHgtControl::FlightStage flight_stage,
                   uint32_t last_heartbeat_ms,
                   uint32_t last_gps_fix_ms) {

    uint32_t now = hal.scheduler->millis();

    //always ignore failsafes in MANUAL modes
    if (mode == NPS_MANUAL) {
        _current_fs_state = NO_FS;
        return;
    }

    //The failsafes are ignored during takeoff and final landing approach
    if (flight_stage == AP_SpdHgtControl::FLIGHT_TAKEOFF || 
        flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
        _current_fs_state = NO_FS;
        return;
    }

    //always check loss of GPS first
    if ((now - last_gps_fix_ms) > 20000) {
        if (_current_fs_state != GPS_LONG_FS) {
            hal.console->println_P(PSTR("20 sec GPS FS"));
        }

        _current_fs_state = GPS_LONG_FS;

        //actually killing throttle is handled in the
        //get_kill_throttle method
        //and the Arduplane code (see Attitude.pde)
        return;
    } else if ((now - last_gps_fix_ms) >= 5000) {
        if (_current_fs_state != GPS_SHORT_FS) {
            hal.console->println_P(PSTR("5 sec GPS FS"));
        }

        _current_fs_state = GPS_SHORT_FS;
       return; 
    } else {
        //GPS is not failing
        //If it was before we have to exit circle mode if we were in it.
        if (_current_fs_state == GPS_LONG_FS 
            || _current_fs_state == GPS_SHORT_FS 
            || _current_fs_state == GPS_RECOVERING_FS) {
            
            if (mode == NPS_CIRCLE) {
                //signal that we probably need to go back to previous flight mode
                _current_fs_state = GPS_RECOVERING_FS;
                return;
            }
        }
    }

    //if we made it here then no failsafes are in effect.
    _current_fs_state = NO_FS;
}

AP_NPS::FailsafeState AP_NPS::get_current_fs_state() {
    return _current_fs_state;
}
