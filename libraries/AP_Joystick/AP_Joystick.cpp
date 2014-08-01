#include "AP_Joystick.h"

#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Joystick::var_info[] PROGMEM = {
    // @Param: FS_TIME
    // @Display Name: Joystick Failsafe Time
    // @Description: If nonzero, specifics the amount of time in seconds after not receiving either a PWM value from the joystick or a hearbeat from the GCS before we engage the failsafe.
    // @Units: Seconds 
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FS_TIME", 0, AP_Joystick, _failsafe_timeout, 2),

    // @Param: FS_ACTION
    // @Display Name: Joystick Failsafe Action
    // @Description: Specifies action taken if telemetry link is lost during joystick operation
    // @Values: 0:None,1:RTL,2:Kill_Throttle
    // @User: Advanced
    AP_GROUPINFO("FS_ACTION", 1, AP_Joystick, _failsafe_action, 1),

    // @Param: LINK_DELAY
    // @Display Name: MAVLink Delay
    // @Description: Amount to delay MAVLink packets sending during joystick operation.  This prioritizes the link for joystick use.
    // @Units: Seconds
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LINK_DELAY", 2, AP_Joystick, _mavlink_delay, 5.0f),

    AP_GROUPEND
};

//constructor
AP_Joystick::AP_Joystick() {
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_Joystick::check_failsafe_ok() {
    if (_enabled == 0 || _failsafe_action == NONE || _failsafe_timeout == 0) {
        return true;
    }

    if (hal.scheduler->millis() - _last_heartbeat_ms > (_failsafe_timeout * 1000U)) {
        return false;
    }

    return true;
}

