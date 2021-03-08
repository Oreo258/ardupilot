#pragma once
#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>

class DACCtrl
{
	public:
		DACCtrl(AP_ServoRelayEvents &relayEvents);
		DACCtrl(const DACCtrl&)=delete;
		DACCtrl& operator=(const DACCtrl&)=delete;
		bool init();
		void update();

		enum VehicleStatus{
			Vehicle_None=200,
			Vehicle_stop,
			Vehicle_forward,
			Vehicle_backward
		};
		enum MotorRunMode{
			MotorRunMode_None=100,
			MotorRunMode_manual,
			MotorRunMode_auto,
			MotorRunMode_Brake,
			MotorRunMode_OutBrake,
			MotorRunMode_MotorStop
		};

		void setMotorControlMode(enum MotorRunMode motor_control_mode){_motor_control_mode=motor_control_mode;}
		int getRepeatCount(){return _reapet_count.get();}
		struct SteerCMD{
			uint8_t _ctrl_data[8];
		}_steer_rot_cmd;

		static const struct AP_Param::GroupInfo var_info[];
		AP_ServoRelayEvents &_relayEvents;
	private:
		void MotorStop();
		void sendDataToMotor();
		void sendData();
		void manualControlMode();
		void autoControlMode();
		void autoModeWriteMotorData();
		void manualModeWriteMotorData();
		uint32_t _throttle_data;
		SRV_Channel * throttle_channel;  //throttle
		SRV_Channel * steer_channel;  //steer

		uint16_t _throttle_pwm_out=0;
		uint16_t _throttle_pwm_min=0;
		uint16_t _throttle_pwm_trim=0;
		uint16_t _throttle_pwm_max=0;

		uint16_t _steer_pwm_out=0;
		uint16_t _steer_pwm_min=0;
		uint16_t _steer_pwm_trim=0;
		uint16_t _steer_pwm_max=0;
		AP_HAL::UARTDriver *port_steer;

		AP_Float _max_throttle;
		AP_Float _min_throttle;
		AP_Int8  _reapet_count;
		enum VehicleStatus _vehicle_status;
		enum MotorRunMode _motor_control_mode{MotorRunMode_None};
};