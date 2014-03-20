#ifndef AP_ACS_H__
#define AP_ACS_H__
/* Naval Postgraduate School Aerial Combat Swarms module */

#include <AP_Param.h>
#include <GCS_MAVLink.h>

class AP_ACS {
public: 
    // for holding parameters
	static const struct AP_Param::GroupInfo var_info[];

    AP_ACS();

    //returns true if the hearbeat was from a companion computer
    bool handle_heartbeat(mavlink_message_t* msg);

    //returns true if everything OK.
    //false if RTL should happen
    bool check();
protected:

    AP_Int8 _watch_heartbeat;

    uint32_t _last_computer_heartbeat_ms;
};

#endif // AP_ACS_H__
