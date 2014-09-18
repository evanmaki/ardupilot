#include <AP_ACS.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 #include <stdio.h>
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_ACS::var_info[] PROGMEM = {
    // @Param: WATCH_HB
    // @DisplayName: Watch the payload heartbeat
    // @Description: If this setting is not 0 then the plane will RTL if 
    // it doesn't receive heartbeats from the payload companion computer.
    // @User: Advanced
    AP_GROUPINFO("WATCH_HB",     0, AP_ACS, _watch_heartbeat,    1),

    // @Param: KILL_THR
    // @DisplayName: Force kill throttle
    // @Description: Can be set to 1 in flight to force throttle to 0 in an emergency.
    // @User: Advanced
    AP_GROUPINFO("KILL_THR", 1, AP_ACS, _kill_throttle, 0),


    AP_GROUPEND
};

AP_ACS::AP_ACS() 
    : _last_computer_heartbeat_ms(0)
    , _current_fs_state(NO_FS)
    , _previous_mode(ACS_NONE)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_ACS::handle_heartbeat(mavlink_message_t* msg) {
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(msg, &packet);

    if (packet.type == MAV_TYPE_ONBOARD_CONTROLLER) {
        //Debug ("Got HB from onboard controller.\n");

        _last_computer_heartbeat_ms = hal.scheduler->millis();

        return true;
    }

    return false;
}

AP_Int8 AP_ACS::get_kill_throttle() {
    //always kill throttle in GPS_LONG_FS
    if (_current_fs_state == GPS_LONG_FS) {
        AP_Int8 retVal;
        retVal.set(1);
        return retVal;
    }

    return _kill_throttle;
}

void AP_ACS::set_kill_throttle(AP_Int8 kt) {
    _kill_throttle = kt;
}

// check for failsafe conditions IN PRIORITY ORDER
bool AP_ACS::check(ACS_FlightMode mode, 
        AP_SpdHgtControl::FlightStage flight_stage, uint32_t last_heartbeat_ms,
        uint32_t last_gps_fix_ms, uint32_t last_radio_status_remrssi_ms) {

    uint32_t now = hal.scheduler->millis();

    //always ignore failsafes in MANUAL modes
    if (mode == ACS_MANUAL || mode == ACS_FLY_BY_WIRE_B || mode == ACS_FLY_BY_WIRE_A) {
        _current_fs_state = NO_FS;
        return false;
    }

    //The failsafes are ignored during takeoff and landing approach
    //(failsafes triggered near the ground could cause the plane to veer at
    //very low altitude into personnel or property)
    if (flight_stage == AP_SpdHgtControl::FLIGHT_TAKEOFF ||
        flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH || 
        flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
        _current_fs_state = NO_FS;
        return false;
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
        return false;
    } else if ((now - last_gps_fix_ms) >= 5000) {
        if (_current_fs_state != GPS_SHORT_FS) {
            hal.console->println_P(PSTR("5 sec GPS FS"));
            _previous_mode = mode;
        }

        _current_fs_state = GPS_SHORT_FS;
       return false; 
    } else {
        //GPS is not failing
        //If it was before we have to exit circle mode if we were in it.
        if (_current_fs_state == GPS_LONG_FS 
            || _current_fs_state == GPS_SHORT_FS 
            || _current_fs_state == GPS_RECOVERING_FS) {
            
            if (mode == ACS_CIRCLE) {
                //signal that we need to go back to previous flight mode
                _current_fs_state = GPS_RECOVERING_FS;
                return false;
            }
        }
    }

    //always check loss of GCS comms 2nd (for auto landing)
    //If we haven't had any contact with the GCS for two minutes, then 
    //enter failsafe state
    if (now - last_hearbeat_ms > 120000 ||
        now - last_radio_status_remrssi_ms > 120000) {
        _current_fs_state = GCS_AUTOLAND_FS;
        return false;
    }

    if (_watch_heartbeat != 0 &&
        now - _last_computer_heartbeat_ms > 20000) {
        _current_fs_state = NO_COMPANION_COMPUTER_FS;
        return false;
    }

    //if we made it here then no failsafes are in effect.
    _current_fs_state = NO_FS;

    return true;
}

#if AP_AHRS_NAVEKF_AVAILABLE
void AP_ACS::send_position_attitude_to_payload(AP_AHRS_NavEKF &ahrs, mavlink_channel_t chan) {
    struct Location loc;
    const struct Location home = ahrs.get_home();
    Vector3f eulers;    
    Vector3f posNED;
    Vector3f velNED;
    Vector3f gyroBias;
    ahrs.get_NavEKF().getEulerAngles(eulers);
    ahrs.get_NavEKF().getVelNED(velNED);
    ahrs.get_NavEKF().getLLH(loc);
    ahrs.get_NavEKF().getPosNED(posNED);
    ahrs.get_NavEKF().getGyroBias(gyroBias);

    float pose[4];
    //TODO: eventually these should be quaternions:
    pose[0] = eulers.x; pose[1] = eulers.y; pose[2] = eulers.z;

    float covariance[45];
    //TODO

    mavlink_msg_global_pos_att_ned_cov_send (chan,
                           hal.scheduler->millis(),
                           loc.lat,
                           loc.lng,
                           loc.alt,
                           loc.alt - home.alt,
                           velNED.x*100.f,
                           velNED.y*100.f,
                           velNED.z*100.f,
                           pose,
                           gyroBias.x,
                           gyroBias.y,
                           gyroBias.z,
                           covariance
                           );
}
#endif //AP_AHRS_NAVEKF_AVAILABLE

