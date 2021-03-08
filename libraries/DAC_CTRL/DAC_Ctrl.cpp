#include "DAC_Ctrl.h"
#include <utility>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#define MAX_steer_rang 14500.0
#define Steer_Mode_Pos 0x21
#define Steer_mode_Speed 0x11

extern const AP_HAL::HAL& hal;
const AP_Param::GroupInfo DACCtrl::var_info[] = {

	//_max_speed
	AP_GROUPINFO("MaxVol", 1, DACCtrl,_max_throttle , 4.2),

	//_max_speed _test_steer_value
	AP_GROUPINFO("MinVol", 2, DACCtrl,_min_throttle , 0.2f),

	AP_GROUPINFO("Repeat", 3, DACCtrl,_reapet_count , 1),

	AP_GROUPEND
};

DACCtrl::DACCtrl(AP_ServoRelayEvents &relayEvents)
:_relayEvents(relayEvents)
{
	AP_Param::setup_object_defaults(this, var_info);
}

bool 
DACCtrl::init()
{
	//get throttle channel
	throttle_channel=SRV_Channels::get_channel_for(SRV_Channel::k_throttle);  //left motor reference right pwm channel
	if(throttle_channel==nullptr){
		hal.console->printf("Unable to get the throttle channel\n");
	}
	//get steer throttle channel
	steer_channel=SRV_Channels::get_channel_for(SRV_Channel::k_steering);  //right motor refernce legt pwm channel
	if(steer_channel==nullptr){
		hal.console->printf("Unable to get the steer channel\n");
	}
	//init the steer seril
	port_steer=AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Steer,0); //left motor
	if(port_steer==nullptr){
		hal.console->printf("Unable to get the steer serial control UART\n");
	}
	return true;
}
void 
DACCtrl::update(){
	if(throttle_channel==nullptr){
		hal.console->printf("throttle left out channel is init err\n");
		return;
	}
	if(steer_channel==nullptr){
		hal.console->printf("throttle right out channel is init err\n");
		return;
	}
	//limit the min speed
	if(_min_throttle<=0.0f){
		_min_throttle=0.0f;
	}
	//limit the max speed
	if(_max_throttle<=_min_throttle){
		_max_throttle=_min_throttle;
	}else if(_max_throttle>=5.0f){
		_max_throttle=5.0f;
	}
	//throttle
	_throttle_pwm_out=throttle_channel->get_output_pwm();//30 is throttle left
	_throttle_pwm_trim=throttle_channel->get_trim();
	_throttle_pwm_max=throttle_channel->get_output_max();
	_throttle_pwm_min=throttle_channel->get_output_min();

	//steer
	_steer_pwm_out=steer_channel->get_output_pwm();
	_steer_pwm_trim=steer_channel->get_trim();
	_steer_pwm_max=steer_channel->get_output_max();
	_steer_pwm_min=steer_channel->get_output_min();

	if((_throttle_pwm_out==_throttle_pwm_trim)){
		_vehicle_status=Vehicle_stop;
	}else if ((_throttle_pwm_out>_throttle_pwm_trim)){
		_vehicle_status=Vehicle_forward;
	}else if((_throttle_pwm_out<_throttle_pwm_trim)){
		_vehicle_status=Vehicle_backward;
	}else{}

	switch(_motor_control_mode){
		case MotorRunMode_None:
			MotorStop();
			break;
		case MotorRunMode_manual:
			manualControlMode();
			break;
		case MotorRunMode_auto:
			autoControlMode();
			break;
		case MotorRunMode_Brake:
			break;
		case MotorRunMode_OutBrake:
			break;
		case MotorRunMode_MotorStop:
			MotorStop();
			break;
		default:
			break;
	}
	sendDataToMotor();
}
void 
DACCtrl::MotorStop()
{
	 //30 off
	_relayEvents.do_set_relay(4,0); // 30
	//18 off
	_relayEvents.do_set_relay(3,0);	// 18 forward
	//fs off
	_relayEvents.do_set_relay(5,0);
}

void 
DACCtrl::manualControlMode()
{
	//4: forward
	if(_vehicle_status==Vehicle_forward){
		//30 off
		_relayEvents.do_set_relay(4,0); // 30
		//18 on
		_relayEvents.do_set_relay(3,1);	// 18 forward
		//fs on
		_relayEvents.do_set_relay(5,1);
	}else if(_vehicle_status==Vehicle_backward){
		//18 off
		_relayEvents.do_set_relay(3,0);	// 18
		//30 on
		_relayEvents.do_set_relay(4,1); // 30
		//fs on
		_relayEvents.do_set_relay(5,1);
	}else{
		 //30 off
		_relayEvents.do_set_relay(4,0); // 30
		//18 off
		_relayEvents.do_set_relay(3,0);	// 18 forward
		//fs off
		_relayEvents.do_set_relay(5,0);
	}
}
void 
DACCtrl::autoControlMode()
{
	if(_vehicle_status==Vehicle_forward){
		//30 off aux-5-->back
		_relayEvents.do_set_relay(4,0); // 30
		//18 on aux-4-->forward
		_relayEvents.do_set_relay(3,1);	// 18 forward
		//fs on  aux-6-->fs
		_relayEvents.do_set_relay(5,1);
	}else if(_vehicle_status==Vehicle_backward){
		//18 off
		_relayEvents.do_set_relay(3,0);	// 18
		//30 on
		_relayEvents.do_set_relay(4,1); // 30
		//fs on
		_relayEvents.do_set_relay(5,1);
	}else {
		 //30 off
		_relayEvents.do_set_relay(4,0); // 30
		//18 off
		_relayEvents.do_set_relay(3,0);	// 18 forward
		//fs off
		_relayEvents.do_set_relay(5,0);
	}
}

void 
DACCtrl::sendDataToMotor()
{
	//throttle
	uint16_t throttle_pwm_out=_throttle_pwm_out;
	if(throttle_pwm_out==_throttle_pwm_trim){
		_throttle_data=0;
	}else if(throttle_pwm_out > _throttle_pwm_trim){ //vehicle forward  need ,motor run - ,so send run - command
		_throttle_data=(uint32_t)13107*(_min_throttle+(((1.0*throttle_pwm_out-1.0*_throttle_pwm_trim)/(1.0*_throttle_pwm_max-1.0*_throttle_pwm_trim))*(_max_throttle-_min_throttle)));
	}else if(throttle_pwm_out < _throttle_pwm_trim){//left motor run +,vehicle back
		_throttle_data=(uint32_t)13107*(_min_throttle+(((1.0*_throttle_pwm_trim-1.0*throttle_pwm_out)/(1.0*_throttle_pwm_trim-1.0*_throttle_pwm_min))*(_max_throttle-_min_throttle)));
	}

	//steer
	uint16_t steer_pwm_out=_steer_pwm_out;
	//clear
	_steer_rot_cmd._ctrl_data[0]=Steer_Mode_Pos; //pos mode
	int32_t ctrl_data=0;
	if(steer_pwm_out==_steer_pwm_trim){
		ctrl_data=0;
	}else if(steer_pwm_out>_steer_pwm_trim){ //cw <0
		ctrl_data=(int32_t)(-1*((1.0*steer_pwm_out-1.0*_steer_pwm_trim)/(1.0*_steer_pwm_max-1.0*_steer_pwm_trim))*MAX_steer_rang);
	}else if(steer_pwm_out<_steer_pwm_trim){ //ccw >0
		ctrl_data=(int32_t)(((1.0*_steer_pwm_trim-1.0*steer_pwm_out)/(1.0*_steer_pwm_trim-1.0*_steer_pwm_min))*MAX_steer_rang);
	}
	_steer_rot_cmd._ctrl_data[4]=(uint8_t)((ctrl_data>>24)&0x000000FF);
	_steer_rot_cmd._ctrl_data[5]=(uint8_t)((ctrl_data>>16)&0x000000FF);
	_steer_rot_cmd._ctrl_data[6]=(uint8_t)((ctrl_data>>8)&0x000000FF);
	_steer_rot_cmd._ctrl_data[7]=(uint8_t)(ctrl_data&0x000000FF);

	//send data
	sendData();
}

void 
DACCtrl::sendData()
{
	//send the steer data
	//send to seril
	if(port_steer!=nullptr){
		port_steer->write((uint8_t*)&_steer_rot_cmd,sizeof(_steer_rot_cmd));
	}

	//send to DAC 
	uint32_t control_data=0;
	control_data=_throttle_data<<8;
	// 1:sclk 2:DI; 3:cs
	_relayEvents.do_set_relay(0,0);
	_relayEvents.do_set_relay(1,1);
	_relayEvents.do_set_relay(2,1);
	hal.scheduler->delay_microseconds(30*1000);//30ms
	_relayEvents.do_set_relay(2,0);//cs low
	uint32_t test2=control_data;
	for(int i=0;i<24;i++){
		if(test2&0x80000000){
			_relayEvents.do_set_relay(1,1);		
		}else{
			_relayEvents.do_set_relay(1,0);
		}
		_relayEvents.do_set_relay(0,1);
		_relayEvents.do_set_relay(0,0);
		test2=test2<<1;
	}
	_relayEvents.do_set_relay(2,1);//cs hight
}