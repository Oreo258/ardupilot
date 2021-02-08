#include "crc.h"

const uint16_t polynom = 0xA001; 
uint16_t CRC16_MODBUS(const control_struct_t& control_data){
	uint8_t temp_bayte[6];
	temp_bayte[0]=control_data.device_address;
	temp_bayte[1]=control_data.function;
	temp_bayte[2]=(uint8_t)(control_data.memory_address>>8);
	temp_bayte[3]=(uint8_t)control_data.memory_address;
	temp_bayte[4]=(uint8_t)(control_data.control_data>>8);
	temp_bayte[5]=(uint8_t)control_data.control_data;

	return crc16bitbybit(temp_bayte,6);
}

uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len){	
	uint8_t i;	
	uint16_t crc = 0xffff; 	
	if (len == 0) {		
		len = 1;	
	}
	while (len--){
		crc ^= *ptr;
		for (i = 0; i<8; i++){
			if (crc & 1) {
				crc >>= 1;
				crc ^= polynom;
			}else {
				crc >>= 1;
			}
		}
		ptr++;	
	}
	return(crc);
}