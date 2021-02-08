#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "SerialControl.h"
#include "crc.h"
extern const AP_HAL::HAL& hal;

SerialControl* SerialControl::_singleton=nullptr;
const AP_Param::GroupInfo SerialControl::var_info[] = {

	//throttle esc address
	AP_GROUPINFO("ThrAddr",	0, SerialControl,throttle_addr , 1),

	//steer esc address
	AP_GROUPINFO("SteAddr",	1, SerialControl,steer_addr , 1),

	//_max_speed
	AP_GROUPINFO("MaxSpeed", 2, SerialControl,_max_speed , 3000.0f),

	//_max_speed _test_steer_value
	AP_GROUPINFO("MinSpeed", 3, SerialControl,_min_speed , 10.0f),

	//AP_GROUPINFO("TestSteer", 4, SerialControl,_test_steer_value , -1000.0f),

	AP_GROUPINFO("StopLat", 4, SerialControl,_stop_point_lat,0.0f),

	AP_GROUPINFO("StopLon", 5, SerialControl,_stop_point_lon,0.0f),

	AP_GROUPINFO("StopSpeed", 6, SerialControl,_stop_speed,0.0f),

	AP_GROUPEND
};

SerialControl::SerialControl(){
	AP_Param::setup_object_defaults(this, var_info);
	if(_singleton!=nullptr){
		return ;
	}
	_singleton=this;

}
bool 
SerialControl::init(){
	_min_speed=10.0f;

	port_throttle=AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_serialControl_throttle,0); //left motor
	if(port_throttle==nullptr){
		hal.console->printf("Unable to get the throttle serial control UART\n");
	}
	//steer
	port_steer=AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_serialControl_steer,0);   //right motor
	if(port_steer==nullptr){
		hal.console->printf("Unable to get the steer serial control UART\n");
	}
	//get throttle channel
	throttle_channel=SRV_Channels::get_channel_for(SRV_Channel::k_throttleLeft);  //left motor reference right pwm channel
	if(throttle_channel==nullptr){
		hal.console->printf("Unable to get the throttle left channel\n");
	}
	//get steer throttle channel
	steer_channel=SRV_Channels::get_channel_for(SRV_Channel::k_throttleRight);  //right motor refernce legt pwm channel
	if(steer_channel==nullptr){
		hal.console->printf("Unable to get the throttle right channel\n");
	}
	return true;
}

void 
SerialControl::MotorStop(){
	//set steer to stop mode
	control_struct_t cmd_data;
	cmd_data.function=FUNCTION_WRITE_DATA;
	//cmd_data.memory_address=MEMORY_ADDRESS_0X0066;
	//cmd_data.control_data=CONTROL_DATA_0X0000;   //run stop
	cmd_data.memory_address=MEMORY_ADDRESS_0X0056;
	cmd_data.control_data=0;
	cmd_data.device_address=throttle_addr;
	cmd_data.crc=CRC16_MODBUS(cmd_data);
	sendThrottleleft(cmd_data);

	//set right stop mode 
	cmd_data.device_address=steer_addr;
	cmd_data.crc=CRC16_MODBUS(cmd_data);
	sendThrottleRight(cmd_data);
}

void 
SerialControl::motorBreak(){
	//set throttle to break mode
	control_struct_t cmd_data;
	cmd_data.device_address=throttle_addr;
	cmd_data.function=FUNCTION_WRITE_DATA;
	cmd_data.memory_address=MEMORY_ADDRESS_0X0066;
	cmd_data.control_data=CONTROL_DATA_0X0003;   //run brake
	cmd_data.crc=CRC16_MODBUS(cmd_data);
	sendThrottleleft(cmd_data);

	//set right break mode 
	cmd_data.device_address=steer_addr;
	cmd_data.crc=CRC16_MODBUS(cmd_data);
	sendThrottleRight(cmd_data);
}
void
SerialControl::motorOutBreak(){
	//set throttle to exit break mode,send 3 times
	control_struct_t cmd_data;
	cmd_data.device_address=throttle_addr;
	cmd_data.function=FUNCTION_WRITE_DATA;
	cmd_data.memory_address=MEMORY_ADDRESS_0X0066;
	cmd_data.control_data=CONTROL_DATA_0X0004;   //run out brake
	cmd_data.crc=CRC16_MODBUS(cmd_data);
	sendThrottleleft(cmd_data);

	//set right break mode 
	cmd_data.device_address=steer_addr;
	cmd_data.crc=CRC16_MODBUS(cmd_data);
	sendThrottleRight(cmd_data);
}


void 
SerialControl::update(){
	if(throttle_channel==nullptr){
		hal.console->printf("throttle left out channel is init err\n");
		return ;
	}
	if(steer_channel==nullptr){
		hal.console->printf("throttle right out channel is init err\n");
		return;
	}

	//get left motor channel value
	_throttle_pwm_out=throttle_channel->get_output_pwm();//30 is throttle left
	_throttle_pwm_trim=throttle_channel->get_trim();
	_throttle_pwm_max=throttle_channel->get_output_max();
	_throttle_pwm_min=throttle_channel->get_output_min();

	_steer_pwm_out=steer_channel->get_output_pwm();
	_steer_pwm_trim=steer_channel->get_trim();
	_steer_pwm_max=steer_channel->get_output_max();
	_steer_pwm_min=steer_channel->get_output_min();

	if((_throttle_pwm_out==_throttle_pwm_trim)&&(_steer_pwm_out==_steer_pwm_trim)){
		_vehicle_status=Vehicle_stop;
	}else if ((_throttle_pwm_out>_throttle_pwm_trim)&&(_steer_pwm_out>_steer_pwm_trim)){
		_vehicle_status=Vehicle_forward;
	}else if((_throttle_pwm_out<_throttle_pwm_trim)&&(_steer_pwm_out<_steer_pwm_trim)){
		_vehicle_status=Vehicle_backward;
	}else{}
	//limit the min speed
	if(_min_speed<=MOTOR_MIN_SPEED){
		_min_speed=MOTOR_MIN_SPEED;
	}
	//limit the max speed
	if(_max_speed<=_min_speed){
		_max_speed=_min_speed;
	}else if(_max_speed>=MOTOR_MAX_SPEED){
		_max_speed=MOTOR_MAX_SPEED;
	}

	switch(_motor_control_mode){
		case MotorRunMode_None:
			return;
		case MotorRunMode_manual:
			manualControlMode();
			break;
		case MotorRunMode_auto:
			autoControlMode();
			break;
		case MotorRunMode_Brake:
			motorBreak();
			break;
		case MotorRunMode_OutBrake:
			motorOutBreak();
			break;
		case MotorRunMode_MotorStop:
			MotorStop();
			break;
		default:
			return;
	}
}

void 
SerialControl::manualControlMode(){
	manualModeWriteThrottleData();
	manualModeWriteSteerData();

}
//vehicle Do not backword 
void 
SerialControl::autoControlMode(){
	autoModeWriteThrottleData();
	autoModeWriteSteerData();
}

//left motor
void 
SerialControl::autoModeWriteThrottleData(){  //left motor,right pwm server

	uint16_t out_pwm_value=_throttle_pwm_out;
	uint16_t throttle_trim=_throttle_pwm_trim;
	uint16_t throttle_max=_throttle_pwm_max;
	memset(&throttle_data,0,sizeof(throttle_data));
	if(out_pwm_value==throttle_trim){
		if((_vehicle_status==Vehicle_stop)||(_vehicle_status==Vehicle_backward)){ //vehicl at stop status 
			throttle_data.control_data=0;
		}else if(_vehicle_status==Vehicle_forward){
			throttle_data.control_data=-_min_speed; //steer motor is run -,laset status: vehicle is forward, now status: vehicle forward turn left 
		}else{
			throttle_data.control_data=0;
		}
	}else if(out_pwm_value > throttle_trim){ //vehicle forward  need ,motor run - ,so send run - command
			throttle_data.control_data = -1*((int16_t)_min_speed+(int16_t)(((1.0*out_pwm_value-1.0*throttle_trim)/(1.0*throttle_max-1.0*throttle_trim))*(_max_speed-_min_speed)));
	}else if(out_pwm_value < throttle_trim){//left motor run +,vehicle back
			throttle_data.control_data=-_min_speed;
	}
	//device address
	throttle_data.device_address= throttle_addr;
	//function
	throttle_data.function=FUNCTION_WRITE_DATA;
	//memory address
	throttle_data.memory_address=MEMORY_ADDRESS_0X0056;
	//crc
	throttle_data.crc=CRC16_MODBUS(throttle_data);

	sendThrottleleft(throttle_data);
}
//right motor
void 
SerialControl::autoModeWriteSteerData(){  //right motor,left pwm sever

	memset(&steer_data,0,sizeof(steer_data));
	//get the control data
	//31 is throttle right
	uint16_t out_pwm_value=_steer_pwm_out;
	uint16_t throttle_trim=_steer_pwm_trim;
	uint16_t throttle_max=_steer_pwm_max;
	if(out_pwm_value==throttle_trim){
		if((_vehicle_status==Vehicle_stop)||(_vehicle_status==Vehicle_backward)){ //vehicl at stop status 
			steer_data.control_data=0;
		}else if(_vehicle_status==Vehicle_forward){ //last status vehicle run forword,now status turn forwar right
			steer_data.control_data=-_min_speed;
		}else{
			steer_data.control_data=0;
		}
	}else if(out_pwm_value > throttle_trim){ //vehicle forword , need right motor run -	
		steer_data.control_data=-1*((int16_t)_min_speed+(int16_t)(((1.0*out_pwm_value-1.0*throttle_trim)/(1.0*throttle_max-1.0*throttle_trim))*(_max_speed-_min_speed)));
	}else if(out_pwm_value < throttle_trim){ //vehicle back, need right motor run +
		steer_data.control_data=-_min_speed;
	}
	//device address
	steer_data.device_address= steer_addr;
	//function
	steer_data.function=FUNCTION_WRITE_DATA;
	//memory address
	steer_data.memory_address=MEMORY_ADDRESS_0X0056;
	//crc
	steer_data.crc=CRC16_MODBUS(steer_data);
	//
	sendThrottleRight(steer_data);
}

//left motor
void 
SerialControl::manualModeWriteThrottleData(){
	uint16_t out_pwm_value=_throttle_pwm_out;
	uint16_t trim=_throttle_pwm_trim;
	uint16_t min=_throttle_pwm_min;
	uint16_t max=_throttle_pwm_max;
	memset(&throttle_data,0,sizeof(throttle_data));
	if(out_pwm_value==trim){
		if(_vehicle_status==Vehicle_stop){
			throttle_data.control_data=0;
		}else if(_vehicle_status==Vehicle_forward){
			throttle_data.control_data=-_min_speed;
		}else if(_vehicle_status==Vehicle_backward){
			throttle_data.control_data=_min_speed;
		}else{
			//never run this point
			throttle_data.control_data=0;	
		}
	}else if(out_pwm_value>trim){
		if(_vehicle_status==Vehicle_backward){
			throttle_data.control_data=(int16_t)_min_speed;
		}else{
			throttle_data.control_data=-1*((int16_t)_min_speed+(int16_t)(((1.0*out_pwm_value-1.0*trim)/(1.0*max-1.0*trim))*(_max_speed-_min_speed)));
		}
	}else if(out_pwm_value<trim){
		if(_vehicle_status==Vehicle_forward){
			throttle_data.control_data=-_min_speed;
		}else {
			throttle_data.control_data=((int16_t)_min_speed+(int16_t)(((1.0*trim-1.0*out_pwm_value)/(1.0*trim-1.0*min))*(_max_speed-_min_speed)));
		}
	}
	//device address
	throttle_data.device_address= throttle_addr;
	//function
	throttle_data.function=FUNCTION_WRITE_DATA;
	//memory address
	throttle_data.memory_address=MEMORY_ADDRESS_0X0056;
	//crc
	throttle_data.crc=CRC16_MODBUS(throttle_data);
	sendThrottleleft(throttle_data);
}
//right motor
void 
SerialControl::manualModeWriteSteerData(){
	//get the control data
	//31 is throttle right
	uint16_t out_pwm_value=_steer_pwm_out;
	uint16_t trim=_steer_pwm_trim;
	uint16_t min=_steer_pwm_min;
	uint16_t max=_steer_pwm_max;
	memset(&steer_data,0,sizeof(steer_data));
	if(out_pwm_value==trim){
		if(_vehicle_status==Vehicle_stop){
			steer_data.control_data=0;
		}else if(_vehicle_status==Vehicle_forward){
			steer_data.control_data=-_min_speed;	
		}else if(_vehicle_status==Vehicle_backward){
			steer_data.control_data=_min_speed;	
		}else{
			steer_data.control_data=0;
		}
	}else if(out_pwm_value>trim){
		if(_vehicle_status==Vehicle_backward){
			steer_data.control_data=_min_speed;	
		}else{
			steer_data.control_data=-1*((int16_t)_min_speed+(int16_t)(((1.0*out_pwm_value-1.0*trim)/(1.0*max-1.0*trim))*(_max_speed-_min_speed)));
		}

	}else if(out_pwm_value<trim){
		if(_vehicle_status==Vehicle_forward){
			steer_data.control_data=-_min_speed;	
		}else{
			steer_data.control_data=((int16_t)_min_speed+(int16_t)(((1.0*trim-1.0*out_pwm_value)/(1.0*trim-1.0*min))*(_max_speed-_min_speed)));
		}	
	}
	//device address
	steer_data.device_address= steer_addr;
	//function
	steer_data.function=FUNCTION_WRITE_DATA;
	//memory address
	steer_data.memory_address=MEMORY_ADDRESS_0X0056;
	//crc
	steer_data.crc=CRC16_MODBUS(steer_data);
	//
	sendThrottleRight(steer_data);
}


void 
SerialControl::setSpeedMode(control_struct_t& cmd_data){
	cmd_data.memory_address=MEMORY_ADDRESS_0X0049;
	cmd_data.control_data=CONTROL_DATA_SPEED_MODE;
}
void 
SerialControl::sendThrottleleft(const control_struct_t& thtottle_left_data){
	if(port_throttle==nullptr){
		hal.console->printf("throttle left uart is init err\n");
		return ;
	}
	uint8_t temp_bayte[8];
	temp_bayte[0]=thtottle_left_data.device_address;
	temp_bayte[1]=thtottle_left_data.function;
	temp_bayte[2]=(uint8_t)(thtottle_left_data.memory_address>>8); //big end
	temp_bayte[3]=(uint8_t)thtottle_left_data.memory_address;
	temp_bayte[4]=(uint8_t)(thtottle_left_data.control_data>>8);
	temp_bayte[5]=(uint8_t)thtottle_left_data.control_data;
	temp_bayte[6]=(uint8_t)thtottle_left_data.crc; //little end
	temp_bayte[7]=(uint8_t)(thtottle_left_data.crc>>8);
	//write to device
	port_throttle->write(temp_bayte,8);
}
void 
SerialControl::sendThrottleRight(const control_struct_t& thtottle_right_data){
	if(port_steer==nullptr){
		hal.console->printf("throttle right uart init err\n");
		return;
	}
	uint8_t temp_bayte[8];
	temp_bayte[0]=thtottle_right_data.device_address;
	temp_bayte[1]=thtottle_right_data.function;
	temp_bayte[2]=(uint8_t)(thtottle_right_data.memory_address>>8); //big end
	temp_bayte[3]=(uint8_t)thtottle_right_data.memory_address;
	temp_bayte[4]=(uint8_t)(thtottle_right_data.control_data>>8);
	temp_bayte[5]=(uint8_t)thtottle_right_data.control_data;
	temp_bayte[6]=(uint8_t)thtottle_right_data.crc; //little end
	temp_bayte[7]=(uint8_t)(thtottle_right_data.crc>>8);
	port_steer->write(temp_bayte,8);
}