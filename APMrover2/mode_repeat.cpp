#include "mode.h"
#include "Rover.h"

bool ModeRepeat::_enter()
{
	// fail to enter auto if no mission commands
    if (rover.mode_auto.get_mision().num_commands() <= 1) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Need set a End Point");
        return false;
    }
	//get the end point
	AP_Mission::Mission_Command cmd;
	//get the first mission waypoint,not home 
	rover.mode_auto.get_mision().get_next_nav_cmd(1,cmd);
	_destination_end=cmd.content.location;
	gcs().send_text(MAV_SEVERITY_NOTICE, "desired end point lat=%f,lon=%f",(float)_destination_end.lat*1e-7,(float)_destination_end.lng*1e-7);
	// set desired location to reasonable stopping point
    if (!g2.wp_nav.set_desired_location_to_stopping_location()) {
		gcs().send_text(MAV_SEVERITY_NOTICE, "set desired location is err!!");
        return false;
    }
    // initialise waypoint speed
    g2.wp_nav.set_desired_speed_to_default();

	_destination_start= rover.current_loc;
	_move_direction=MoveDirection_Forword;
	_count=0;
	gcs().send_text(MAV_SEVERITY_NOTICE, "Enter Reapet Mode");
	return set_desired_location(_destination_end);
}
void ModeRepeat::_exit()
{
	g2._dac.setMotorControlMode(DACCtrl::MotorRunMode_None);
	g2.wp_nav.set_reversed(false);// forward
}
void ModeRepeat::update()
{
	g2._dac.setMotorControlMode(DACCtrl::MotorRunMode_auto);
	switch (_move_direction) {
        case MoveDirection_Forword:
        {
            // check if we've reached the destination
            if (!g2.wp_nav.reached_destination()) {
				g2.wp_nav.set_reversed(false);// forward
                // update navigation controller
                navigate_to_waypoint();
            } else {
				if(set_desired_location(_destination_start)){
					_move_direction=MoveDirection_BACK;
				}
            }
            break;
        }
		case MoveDirection_BACK:
		{
			if(!g2.wp_nav.reached_destination()){
				g2.wp_nav.set_reversed(true);// back to
				navigate_to_waypoint();
			}else{
				_count++;
				if(_count>=g2._dac.getRepeatCount()){
					stop_vehicle();
				}else{
					if(set_desired_location(_destination_end)){
						_move_direction=MoveDirection_Forword;
					}
				}
			}
		}
			break;
		default:
			break;
    }
}