// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Land.h
/// @brief   Class for storing land parameters and methods.  Created to make mission planning for landing more simple.

/*
 * The AP_Land library:
 * 
 * Initial implementation: Michael Day, April 2014
 */
#ifndef AP_Land_H
#define AP_Land_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_AHRS.h>
#include <Compass.h>
#include <AP_TECS.h>
#include <AP_Rally.h>
#include <AP_Mission.h>

class AP_Land {
public:
    AP_Land(AP_AHRS &ahrs, Compass &compass, AP_TECS &tecs, AP_Rally &rally,
            AP_Mission &mission);
    
    uint8_t get_land_wings_level() const { return _land_wing_level; }

    // Called periodically -- moves through each step of the landing sequence.
    // Rally glide landing method.
    // Returns false on error.
    // Before calling: Enter RTL mode (to begin proceeding to a rally point).
    bool preland_step_rally_land(const RallyLocation &ral_loc);

    //Called prior to starting landing sequnece.  
    void preland_init();

    //true if we've started the pre-landing sequend.  false otherwise.
    bool preland_started() const { return _preland_started; }

    void abort_landing();

    Location get_location_1km_beyond_land() const;

    //return -1 if no landing waypoint could be found in the Mission or 
    //if the landing waypoint is too far away from base_loc
    int16_t find_nearest_landing_wp_index(const Location& base_loc) const;
    
    //Caller needs to know when to head to break altitude b/c I can't control
    //the current waypoint altitude from this library.
    bool head_to_break_alt() const { return _head_to_break_alt; }

    bool heading_as_desired_for_landing() const { return _land_heading_as_desired; }
    bool speed_as_desired_for_landing() const { return _land_speed_as_desired; }
    bool arrived_at_break_alt() const { return _land_break_alt_as_desired; }

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

protected:
    const AP_AHRS& _ahrs;
    const Compass& _compass;
    const AP_TECS& _tecs;
    const AP_Rally& _rally;
    const AP_Mission& _mission;

    int16_t _landing_wp_index;
    Location _landing_wp;

    bool _preland_started;
    bool _head_to_break_alt;
    bool _land_break_alt_as_desired;
    bool _land_heading_as_desired;
    bool _land_speed_as_desired;

    //parameters
    AP_Int8 _land_wing_level;
};

#endif //AP_Land_H
