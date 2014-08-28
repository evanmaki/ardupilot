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
    uint8_t get_reset_stream_ticks() { return _reset_stream_ticks; }
    float get_mavlink_delay() { return _mavlink_delay; }

    void set_enabled(uint8_t enable) { _enabled = enable; }
    void set_reset_stream_ticks(uint8_t reset_stream_ticks) { _reset_stream_ticks = reset_stream_ticks; }
   
    //parameter block
    static const struct AP_Param::GroupInfo var_info[];

protected:

    //parameters
    AP_Int8 _enabled;
    AP_Int8 _reset_stream_ticks;
    AP_Float _mavlink_delay;
    
};

#endif //AP_Joystick_h
