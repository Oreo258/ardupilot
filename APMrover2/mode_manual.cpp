#include "mode.h"
#include "Rover.h"

bool ModeManual::_enter()
{
    _try_control=false;
    _try_start_time_ms=0;
    return true;
}

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
    g2.serial_control.setMotorControlMode(SerialControl::MotorRunMode_None);
    _try_control=false;
    _try_start_time_ms=0;
}

void ModeManual::update()
{
    float desired_steering=0.0f; 
    float desired_throttle=0.0f;// desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

    g2.serial_control.setMotorControlMode(SerialControl::MotorRunMode_manual);

    //try the motor
    if((rover.channel_throttle->get_control_in()==0)&&(rover.channel_steer->get_control_in()==0)){
        if(_try_control){
            if(_try_start_time_ms==0){
                _try_start_time_ms=AP_HAL::millis();
            }

            if(AP_HAL::millis()-_try_start_time_ms<3000){
                desired_throttle=10.0f;
            }else if((AP_HAL::millis()-_try_start_time_ms>=3000)&&(AP_HAL::millis()-_try_start_time_ms<6000)){
                desired_throttle=-10.0f;
            }else{
                _try_control=false;
                _try_start_time_ms=0;
            }
        }else{
            if(GCS_MAVLINK::_command_from_gcs.move_diret==1){
                desired_throttle=GCS_MAVLINK::_command_from_gcs.thr;
            }else if(GCS_MAVLINK::_command_from_gcs.move_diret==2){
                desired_throttle=-GCS_MAVLINK::_command_from_gcs.thr;
            }
        }
    }else{
        _try_control=false;
        _try_start_time_ms=0;
    }

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, false);
}
