#pragma once
#include "BLDC-100A.h"
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
uint16_t CRC16_MODBUS(const control_struct_t& control_data);
uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len);