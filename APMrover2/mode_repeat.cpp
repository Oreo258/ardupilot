#include "mode.h"
#include "Rover.h"

bool ModeRepeat::_enter()
{
    // create new mission command
    AP_Mission::Mission_Command cmd = {};

    // read command at index 1
    if (!rover.mode_auto.mission.read_cmd_from_storage(1, cmd)) {
        return false;
    }
    // it's not a waypoint
    if (cmd.id != MAV_CMD_NAV_WAYPOINT) {
        return false;
    }

    // set destination
    if (!g2.wp_nav.set_desired_location(cmd.content.location)) {
        return false;
    }

    // initialise waypoint speed
    g2.wp_nav.set_desired_speed_to_default();

    repeat = false;
    repeat_count = 0;
    last_stop_time_ms = 0;

    return true;
}

void ModeRepeat::update()
{
    // determine if we should keep navigating
    if (!g2.wp_nav.reached_destination()) {
        // update navigation controller
        navigate_to_waypoint();
    } else {

        // we have reached the destination
        // boats loiter, rovers stop
    if (stop_vehicle()) {
            if (repeat_count < g2.round_max) {
                const uint32_t now_ms = AP_HAL::millis();

                if (last_stop_time_ms == 0) {
                    last_stop_time_ms = now_ms;
                }

                if (now_ms - last_stop_time_ms > g2.round_delay * 1000) {
                    if (!repeat) {
                        // set destination
                        if (!g2.wp_nav.set_desired_location(origin_loc)) {
                            //return;
                        }
                        set_reversed(true);
                    } else {
                        gcs().send_text(MAV_SEVERITY_INFO, "Repeat %d finished", unsigned(repeat_count + 1));
                        if (++repeat_count < g2.round_max) {
                            // set destination
                            if (!g2.wp_nav.set_desired_location(destination_loc)) {
                                //return;
                            }
                            set_reversed(false);
                        }
                    }
                    repeat = !repeat;
                    last_stop_time_ms = 0;
                }
            }
    }

        // update distance to destination
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
    }
}

// get desired location
bool ModeRepeat::get_desired_location(Location& destination) const
{
    if (g2.wp_nav.is_destination_valid()) {
        destination = g2.wp_nav.get_oa_destination();
        return true;
    }
    return false;
}

// set desired speed in m/s
bool ModeRepeat::set_desired_speed(float speed)
{
    if (is_negative(speed)) {
        return false;
    }
    g2.wp_nav.set_desired_speed(speed);
    return true;
}