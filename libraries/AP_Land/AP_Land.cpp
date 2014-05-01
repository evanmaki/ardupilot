// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Land.cpp
/// @brief   Class for storing land parameters and methods.  Created to make mission planning for landing more simple.
#include "AP_Land.h"

#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 #include <stdio.h>
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

const AP_Param::GroupInfo AP_Land::var_info[] PROGMEM = {
    // @Param: WING_LEVEL
    // @DisplayName: Land Wings Level
    // @Description: How wings are held at landing after flare.  Some planes want to only navigate with the rudder when close to the ground to avoid the wing tips scraping.  Other planes don't have a rudder and don't have this option.  Still other planes have small enough wings that they may or may not want to keep wings level during the final landing phase. 
    // @Values: 0:KeepNavigating,1:HoldLevel,
    // @User: Advanced
    AP_GROUPINFO("WING_LEVEL", 0, AP_Land, _land_wing_level, 1),

    AP_GROUPEND
};

//constructor
AP_Land::AP_Land(AP_AHRS &ahrs, Compass &compass, AP_TECS &tecs, AP_Rally &rally, AP_Mission &mission, AP_InertialSensor &ins)
    : _ahrs(ahrs)
    , _compass(compass)
    , _tecs(tecs)
    , _rally(rally)
    , _mission(mission)
    , _ins(ins)
    , _landing_wp_index(-1)
    , _preland_started(false)
    , _head_to_break_alt(false)
    , _land_break_alt_as_desired(false)
    , _land_heading_as_desired(false)
    , _land_speed_as_desired(false) 
    , _num_accel_data_points(0) 
    , _current_accel_idx(0) {

    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Land::preland_init() {
    _landing_wp_index = -1;
    _preland_started = true;
    _head_to_break_alt = false;
    _land_break_alt_as_desired = false;
    _land_heading_as_desired = false;
    _land_speed_as_desired = false;

    _num_accel_data_points = 0;
    _current_accel_idx = 0;
}

void AP_Land::abort_landing() {
    preland_init();

    _preland_started = false;
}

bool AP_Land::still_vibrating() {
    Vector3f accels = _ins.get_accel();
    
    _x_accel_list[_current_accel_idx] = accels.x;
    _y_accel_list[_current_accel_idx] = accels.y;
    _z_accel_list[_current_accel_idx] = accels.z;
    _current_accel_idx++;
    _current_accel_idx %= AUTO_DISARM_SAMPLE_SIZE;

    //don't do this test if you don't have enough samples
    if (_num_accel_data_points < AUTO_DISARM_SAMPLE_SIZE) {
        _num_accel_data_points++;
        return true;
    } 

    //Debug ("Var X: %f Var Y: %f Var Z: %f \n", variance(_x_accel_list), variance(_y_accel_list), variance(_z_accel_list));

    if (variance(_x_accel_list) < 0.001f && variance(_y_accel_list) < 0.001f
            && variance(_z_accel_list) < 0.001f) {
        return false;
    }

    return true;
}

Location AP_Land::get_location_1km_beyond_land() const {
    Location ret_loc = 
        _rally.rally_location_to_location(_rally.get_current_rally_point());

    location_update(ret_loc, get_bearing_cd(ret_loc, _landing_wp) * 0.01f, 
                    get_distance(ret_loc, _landing_wp) + 1000.0f);

    return ret_loc;
}

int16_t AP_Land::find_nearest_landing_wp_index(const Location& base_loc) const {
    // Start minimum distance at appx infinity
    float min_distance = 9999.9;
    int tmp_distance;
    int16_t land_wp_index = -1;
    
    AP_Mission::Mission_Command tmp = {0};

    // Go through all WayPoints looking for landing waypoints
    for(uint16_t i = 0; i< _mission.num_commands()+1; i++) {
        _mission.read_cmd_from_storage(i, tmp);
        if(tmp.id == MAV_CMD_NAV_LAND) {
           tmp_distance = get_distance(tmp.content.location, base_loc);
           if(tmp_distance < min_distance) {
              min_distance = tmp_distance;
              land_wp_index = i;
              break;
           }
        }
    }

    //DON'T GO TOO FAR! if the closest landing waypoint is too far away
    //(based on g.rally_limit_km), then don't use it -- don't autoland.
    if (_rally.get_rally_limit_km() > 0.f && 
            min_distance > _rally.get_rally_limit_km() * 1000.0f) {
        return -1;
    }

    return land_wp_index; 
}

bool AP_Land::preland_step_rally_land(const RallyLocation &ral_loc) {
    //If we haven't started the pre-land sequence, bail with no error.
    if (! _preland_started) {
        return true;
    }

    //If there are no rally points in this mission,
    //then this landing method doesn't make sense.
    if (_rally.get_rally_total() < 1) {
        Debug("No Rally Points.");
        return false;
    }

    Location current_loc;
    ((AP_AHRS&) _ahrs).get_position(current_loc);

    //find the best landing waypoint (if not already found)
    if (_landing_wp_index == -1) {
        _landing_wp_index = 
            find_nearest_landing_wp_index(_rally.rally_location_to_location(ral_loc));
        if (_landing_wp_index == -1) {
            Debug("No suitable suitable Landing Point (too far away?)");
            return false;
        }

        AP_Mission::Mission_Command lnd_cmd;
        _mission.read_cmd_from_storage(_landing_wp_index, lnd_cmd);
        _landing_wp = lnd_cmd.content.location;
    }
     
    //time to head for break altitude?
    if (_head_to_break_alt == false) {        
        //close enough to start breaking?
        if(get_distance(current_loc,_rally.rally_location_to_location(ral_loc))<100.0f) {
            //CAN'T DO THIS from a library.  Hence the head_to_break_alt method.
            //TODO: be able to modify current WP from a library. 
            //next_WP_cmd.content.location.alt = break_alt;
           
            _head_to_break_alt = true;

            /* Since fence isn't a library, we can't auto disable it from here.
             * TODO: Make geofence a library.
             */
        }
    } else { //we should be already heading for the break_alt
        // Check if we're at break_alt + 5 meters or less
        _land_break_alt_as_desired = false;
        if( current_loc.alt < 
                ((ral_loc.break_alt*100UL) + _ahrs.get_home().alt + 500)) {
            _land_break_alt_as_desired = true;
            // Calculate bearing in radians
            float bearing = (radians( (float)(get_bearing_cd(current_loc,_landing_wp)/100.0) ));

            // Calculate heading
            float heading = 5000;
            if (((Compass &) _compass).read()) {
                const Matrix3f &m = _ahrs.get_dcm_matrix();
                heading = _compass.calculate_heading(m);

                //map heading to bearing's coordinate space:
                if (heading < 0.0f) {
                    heading += 2.0f*PI;
                }
            }
 
            // Check to see if the the plane is heading toward the land waypoint
            //3 degrees = 0.0523598776 radians
            _land_heading_as_desired = false;
            if (fabs(bearing - heading) <= 0.0523598776f) {
                _land_heading_as_desired = true;
                
                //now ensure we're going slow enough:
                float aspeed;
                _ahrs.airspeed_estimate(&aspeed);
                _land_speed_as_desired = false;
                if (aspeed - _tecs.get_target_airspeed() <= 2.0f) {
                    _land_speed_as_desired = true;
                                 
                    /*       
                    //Can't set mode from a library!  These calls will
                    //have to be done from elsewhere! 
                    //Hence the heading_as_desired_for_landing and the
                    //speed_as_desired_for_landing methods
                    //
                    //Someday it would be nice to be able to have standardized
                    //modes that are accessible from libs.
                    //
                    //The order of these commands appears to be important
                    _mission.set_current_cmd(rally_land_wp_idx);
                    set_mode(AUTO);
                    mission.resume();
                    */                    
                }
            }
        } 
    }

    return true;
}

float AP_Land::variance(const float list[]) const {
    int i;   
    float sum,m;  
    m=mean(list);  
    sum=0;  
    for(i=0; i<AUTO_DISARM_SAMPLE_SIZE; i++) {
        sum+=pow((list[i]-m),2);  
    }  
    
    return (sum / (float) AUTO_DISARM_SAMPLE_SIZE);
}

float AP_Land::mean(const float list[]) const {
    int i;  
    float sum=0;  
    for(i=0; i<AUTO_DISARM_SAMPLE_SIZE; i++) { 
        sum=sum+list[i];
    }

    return (sum / (float)AUTO_DISARM_SAMPLE_SIZE); 
}
