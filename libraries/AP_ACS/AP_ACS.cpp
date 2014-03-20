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

    AP_GROUPEND
};

AP_ACS::AP_ACS() 
    : _last_computer_heartbeat_ms(0)
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

bool AP_ACS::check() {
    if (_watch_heartbeat != 0 &&
        hal.scheduler->millis() - _last_computer_heartbeat_ms > 20000) {
        return false;
    }

    return true;
}
