/* 
 * Custom module for Naval Postgraudate School (NPS)
 *
 * Currently used for custom NPS failsafes, can be expanded for
 * other custom NPS behavior in the future (if desired)
 *
 * Michael Day, Systems Engineering Dept, Naval Postgraduate School, Feb 2014
 */

#ifndef _APM_NPS_H_
#define _APM_NPS_H_

#include <AP_Param.h>
#include <AP_TECS.h>

class AP_NPS {
public:
    //TODO: modes should be in their own library so they aren't duplicated here.
    //I wonder if AP_Mission was supposed to do that.  It may need some help.
    typedef enum NPS_FlightMode {
        NPS_MANUAL        = 0,
        NPS_CIRCLE        = 1,
        NPS_STABILIZE     = 2,
        NPS_TRAINING      = 3,
        NPS_ACRO          = 4,
        NPS_FLY_BY_WIRE_A = 5,
        NPS_FLY_BY_WIRE_B = 6,
        NPS_CRUISE        = 7,
        NPS_AUTO          = 10,
        NPS_RTL           = 11,
        NPS_LOITER        = 12,
        NPS_GUIDED        = 15,
        NPS_INITIALISING  = 16
    } NPS_FlightMode;

    //note that these are defined in priority order
    typedef enum FailsafeState {
        GPS_LONG_FS = 0,
        GPS_SHORT_FS,
        GPS_RECOVERING_FS,
        SEC_CONTROL_FS,
        BATTERY_CURR_FS,
        BATTERY_VOLT_FS,
        GEOFENCE_FS,
        GCS_FS,
        THROTTLE_FS,
        NO_FS,
    } FailsafeState;

    // for holding parameters
	static const struct AP_Param::GroupInfo var_info[];

    AP_NPS();

    // essentially returns a bool: if not 0, then DO kill throttle
    AP_Int8 get_kill_throttle();

    // essentially setting a bool: if not 0, then DO kill throttle
    void set_kill_throttle(AP_Int8 kt);

    void check(NPS_FlightMode mode, AP_SpdHgtControl::FlightStage flight_stage,
           uint32_t last_heartbeat_ms, uint32_t last_gps_fix_ms);

    FailsafeState get_current_fs_state();

private:
    AP_Int8             _kill_throttle;

    FailsafeState       _current_fs_state;

};
#endif //_APM_NPS_H_
