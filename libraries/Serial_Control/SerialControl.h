#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "BLDC-100A.h"

#define MOTOR_CHANGE_SEND_COUNT 20
class SerialControl{
public:
	SerialControl();
	SerialControl(const SerialControl&)=delete;
	SerialControl& operator=(const SerialControl&)=delete;
	static SerialControl* get_singleton(){return _singleton; }
	enum MotorRunMode{
		MotorRunMode_None=100,
		MotorRunMode_manual,
		MotorRunMode_auto,
		MotorRunMode_Brake,
		MotorRunMode_OutBrake,
		MotorRunMode_MotorStop
	};
	enum VehicleStatus{
		Vehicle_None=200,
		Vehicle_stop,
		Vehicle_forward,
		Vehicle_backward
	};

	bool init();
	void update();
	void MotorStop();
	void motorBreak();
	void motorOutBreak();
	void setSpeedMode(control_struct_t& cmd_data);
	void setMotorControlMode(enum MotorRunMode motor_control_mode){_motor_control_mode=motor_control_mode;}
	enum VehicleStatus getVehicleStatus(){return _vehicle_status;}
	enum MotorRunMode getMotorControlMode(){return _motor_control_mode;}
	void getStopPoint(struct Location& stop_point){stop_point.lat=(int32_t)_stop_point_lat*1e7;stop_point.lng=(int32_t)_stop_point_lon*1e7;stop_point.alt=0;}
	float getStopSpeed(){return _stop_speed;}
	// parameter var table
	static const struct AP_Param::GroupInfo var_info[];

private:
	void manualControlMode();
	void autoControlMode();
	void writeThrottleData();
	void writeSteerData();
	void manualModeWriteThrottleData();
	void manualModeWriteSteerData();
	void autoModeWriteThrottleData();
	void autoModeWriteSteerData();
	void sendThrottleleft(const control_struct_t& thtottle_left_data); //throttle 
	void sendThrottleRight(const control_struct_t& thtottle_right_data);//steer

	static SerialControl* _singleton;
	AP_HAL::UARTDriver *port_throttle;  
	//AP_HAL::UARTDriver *port_steer;  
	AP_HAL::UARTDriver *port_steer;
	AP_Int8 throttle_addr;
	AP_Int8 steer_addr;
	AP_Float _max_speed;
	AP_Float _min_speed;
	AP_Int32 _stop_point_lat;
	AP_Int32 _stop_point_lon;
	AP_Float _stop_speed;
	control_struct_t throttle_data;
	control_struct_t steer_data;
	SRV_Channel * throttle_channel;  //throttle left
	SRV_Channel * steer_channel;     //throttle right

	uint16_t _throttle_pwm_out=0;
	uint16_t _throttle_pwm_min=0;
	uint16_t _throttle_pwm_trim=0;
	uint16_t _throttle_pwm_max=0;

	uint16_t _steer_pwm_out=0;
	uint16_t _steer_pwm_min=0;
	uint16_t _steer_pwm_trim=0;
	uint16_t _steer_pwm_max=0;
	enum MotorRunMode _motor_control_mode{MotorRunMode_None};
	enum VehicleStatus _vehicle_status{Vehicle_None};
};