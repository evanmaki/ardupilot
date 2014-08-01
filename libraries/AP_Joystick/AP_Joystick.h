/*
 * Maintains parameters for joystick operation.
 * Also handles special failsafe for joystick operation: if the telemetry link
 *      is lost then kill throttle.  Note the failsafe is optional
 *
 */
#ifndef AP_Joystick_h
#define AP_Joystick_h

#include <AP_Common.h>
#include <AP_Param.h>

/// @class AP_Joystick
/// @brief Maintains parameters for joystick / game pad operation
class AP_Joystick {
public:
    typedef enum JS_FS_ACTION {
        NONE = 0,
        RTL,
        KILL_THROTTLE
    } JS_FS_ACTION;

    AP_Joystick();

    uint8_t get_enabled() { return _enabled; }
    uint8_t get_failsafe_timeout() { return _failsafe_timeout; }
    uint8_t get_failsafe_action() { return _failsafe_action; }
    float get_mavlink_delay() { return _mavlink_delay; }

    void set_last_heartbeat_ms(uint32_t last_hb_ms) { _last_heartbeat_ms = last_hb_ms; }

    //return true if failsafe ok,
    //false if failsafe should fire
    bool check_failsafe_ok();

    //parameter block
    static const struct AP_Param::GroupInfo var_info[];

protected:

    uint32_t _last_heartbeat_ms;

    //parameters
    AP_Int8 _enabled;
    AP_Int8 _failsafe_timeout;
    AP_Int8 _failsafe_action;
    AP_Float _mavlink_delay;
    
};

#endif //AP_Joystick_h
