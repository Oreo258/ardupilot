#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

class AP_Proximity_DYP:public AP_Proximity_Backend
{
	public:
		AP_Proximity_DYP(AP_Proximity &_frontend,AP_Proximity::Proximity_State &_state);
		
		static bool detect();
		void update(void)override;
		 // get maximum and minimum distances (in meters) of sensor
		float distance_max() const override { return 4.5f; }
		float distance_min() const override { return 0.30f; }
		enum PARSE_phase{
			STEP_head=0,
			STEP_data
		};
	private:
		void initialise();
		void collect_byte(uint8_t msg);
		void parserMsg();
		void update_sector_distance(float angle,float distance);
		enum PARSE_phase _parse_phase{STEP_head};
		uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
		AP_HAL::UARTDriver *_uart=nullptr;              // uart for communicating with sensor
		uint8_t _data[9];
		uint8_t _data_index=0;

};