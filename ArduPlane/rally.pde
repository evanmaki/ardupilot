// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * rally point support
 * Michael Day, September 2013
 */

//get a rally point
static bool get_rally_point_with_index(unsigned i, RallyLocation &ret)
{
    if (i >= (unsigned)g.rally_total) {
        return false;
    }
    hal.storage->read_block(&ret, RALLY_START_BYTE + (i * sizeof(RallyLocation)), sizeof(RallyLocation));
    if (ret.lat == 0 && ret.lng == 0) {
        // sanity check ...
        return false;
    }
    return true; 
}

//save a rally point
static bool set_rally_point_with_index(unsigned i, const RallyLocation &rallyLoc)
{
    if (i >= (unsigned) g.rally_total) {
        //not allowed
        return false;
    }

    if (i >= MAX_RALLYPOINTS) {
        //also not allowed
        return false;
    }
    hal.storage->write_block(RALLY_START_BYTE + (i * sizeof(RallyLocation)), &rallyLoc, sizeof(RallyLocation));
    return true;
}

// 'best' means 'closest to Location loc' for now.
static bool find_best_rally_point(const Location &myloc, const Location &homeloc, RallyLocation &ret) 
{
    float min_dis = -1;

    for (unsigned i = 0; i < (unsigned) g.rally_total; i++) {
        RallyLocation next_rally;
        if (!get_rally_point_with_index(i, next_rally)) {
            continue;
        }
        Location rally_loc = rally_location_to_location(next_rally, homeloc);
        float dis = get_distance(myloc, rally_loc);

        if (dis < min_dis || min_dis < 0) {
            min_dis = dis;
            ret = next_rally;
        }
    }

    if (g.rally_limit_km > 0 && min_dis > g.rally_limit_km*1000.0f && 
        get_distance(myloc, homeloc) < min_dis) {
        // return false, which makes home be used instead
        return false;
    }

    return min_dis >= 0;
}

// translate a RallyLocation to a Location
static Location rally_location_to_location(const RallyLocation &r_loc, const Location &homeloc) 
{
    Location ret = {};

    ret.id = MAV_CMD_NAV_LOITER_UNLIM;
    ret.options = MASK_OPTIONS_RELATIVE_ALT;

    //Currently can't do true AGL on the APM.  Relative altitudes are
    //relative to HOME point's altitude.  Terrain on the board is inbound
    //for the PX4, though.  This line will need to be updated when that happens:
    ret.alt = (r_loc.alt*100UL) + homeloc.alt; 

    ret.lat = r_loc.lat;
    ret.lng = r_loc.lng;

    return ret;
}

// return best RTL location from current position
static Location rally_find_best_location(const Location &myloc, const Location &homeloc)
{
    RallyLocation ral_loc = {};
    Location ret = {};
    if (find_best_rally_point(myloc, home, ral_loc)) {
        //we have setup Rally points: use them instead of Home for RTL
        ret = rally_location_to_location(ral_loc, home);
    } else {
        ret = homeloc;
        // Altitude to hold over home
        ret.alt = read_alt_to_hold();
    }
    return ret;
}

// Finds best landing location based on rally_loc
// returns index of the landing waypoint, or -1 if no suitable location is found
static int16_t find_best_landing_wp(const Location &myloc, Location &landing_WP) {
    // Start minimum distance at appx infinity
    float min_distance = 9999.9;
    int tmp_distance;
    int16_t landing_wp_index = -1;

    // Go through all WayPoints looking for landing waypoints
    for(int16_t i = 0;i<g.command_total+1;i++) {
        Location tmp = get_cmd_with_index(i);
        if(tmp.id == MAV_CMD_NAV_LAND) {
           tmp_distance = get_distance(tmp, myloc);
           if(tmp_distance < min_distance) {
              landing_WP = tmp;
              min_distance = tmp_distance;
              landing_wp_index = i;
           }
        }
    }
    return landing_wp_index;
}

