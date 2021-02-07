#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_Proximity_DYP.h"
#define PROXIMITY_DYP_TIMEOUT_MS 500

extern const AP_HAL::HAL& hal;

AP_Proximity_DYP::AP_Proximity_DYP(AP_Proximity &_frontend,AP_Proximity::Proximity_State &_state)
:AP_Proximity_Backend(_frontend, _state)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Proximity_DYP, 0);
    if (_uart != nullptr) {
        // start uart with larger receive buffer
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Proximity_DYP, 0), 512, 0);
    }
    initialise();
}
bool 
AP_Proximity_DYP::detect()
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Proximity_DYP, 0)!=nullptr;
}
void 
AP_Proximity_DYP::initialise()
{
    // initialise boundary
    init_boundary();
}
void 
AP_Proximity_DYP::update()
{
    if (_uart == nullptr) {
        return;
    }
    //TODO: add the read uart code
    int16_t nbytes = _uart->available();
    hal.console->printf("==>>>>");
    while (nbytes-- > 0) {
        const int16_t r = _uart->read();
        if((r < 0) || (r > 0xFF)){
            continue;
        }
        //hal.console->printf("%02X",(uint8_t)r);
        collect_byte((uint8_t)r);
    }
    hal.console->printf("<<<<==");
    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_DYP_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}
void 
AP_Proximity_DYP::collect_byte(uint8_t msg)
{
    switch(_parse_phase){
        case  STEP_head:
            if(msg==0XFF){
                _parse_phase=STEP_data;
                _data_index=0;
            }
            break;
        case STEP_data:
            _data[_data_index]=msg;
            _data_index++;
            if(_data_index>=9){
                parserMsg();
                _parse_phase=STEP_head;
            }
            break;
        default :
            break;
    }
}
void
AP_Proximity_DYP::parserMsg()
{
    uint16_t check_sum=0XFF;
    for(int i=0;i<8;i++){
        check_sum+=_data[i];
    }
    uint8_t low_check_sum=(uint8_t) check_sum&0x00FF;            
    if(low_check_sum==_data[8]){
        update_sector_distance(315,(_data[0]*256+_data[1])*0.001);  //firt one
        update_sector_distance(0,(_data[2]*256+_data[3])*0.001);
        update_sector_distance(45,(_data[4]*256+_data[5])*0.001);
        //update_sector_distance(90,(_data[6]*256+_data[7])*0.001);
        uint32_t now = AP_HAL::millis();
        _last_distance_received_ms= now;
    }
}
void 
AP_Proximity_DYP::update_sector_distance(float angle,float distance)
{
    const uint8_t sector = convert_angle_to_sector(angle);
    _angle[sector] = angle;
    _distance[sector] = distance;//m
    _distance_valid[sector] = (_distance[sector] >= 0.3f) && (_distance[sector] <= 4.5f);
    update_boundary_for_sector(sector, true);
}