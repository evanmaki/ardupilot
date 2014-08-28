#include "AP_Joystick.h"

#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Joystick::var_info[] PROGMEM = {
    // @Param: LINK_DELAY
    // @Display Name: MAVLink Delay
    // @Description: Amount to delay MAVLink packets sending during joystick operation.  This prioritizes the link for joystick use.
    // @Units: Seconds
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LINK_DELAY", 0, AP_Joystick, _mavlink_delay, 1.0f),

    AP_GROUPEND
};

//constructor
AP_Joystick::AP_Joystick() {
    AP_Param::setup_object_defaults(this, var_info);
}

