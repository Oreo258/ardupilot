#include "siyi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_SIYI_HEADER1       0x55    // first header byte
#define AP_MOUNT_SIYI_HEADER2       0x66    // second header byte
#define AP_MOUNT_SIYI_PACKETLEN_MIN 10      // minimum number of bytes in a packet.  this is a packet with no data bytes
#define AP_MOUNT_SIYI_DATALEN_MAX   (AP_MOUNT_SIYI_PACKETLEN_MAX-AP_MOUNT_SIYI_PACKETLEN_MIN) // max bytes for data portion of packet
#define AP_MOUNT_SIYI_SERIAL_RESEND_MS   1000    // resend angle targets to gimbal once per second
#define AP_MOUNT_SIYI_MSG_BUF_DATA_START 8  // data starts at this byte in _msg_buf
#define AP_MOUNT_SIYI_RATE_MAX_RADS radians(90) // maximum physical rotation rate of gimbal in radans/sec
#define AP_MOUNT_SIYI_PITCH_P       1.50    // pitch controller P gain (converts pitch angle error to target rate)
#define AP_MOUNT_SIYI_YAW_P         1.50    // yaw controller P gain (converts yaw angle error to target rate)
#define AP_MOUNT_SIYI_LOCK_RESEND_COUNT 5   // lock value is resent to gimbal every 5 iterations

Transfer* Transfer::_singleton=nullptr;
Transfer::Transfer()
{
    _singleton=this;
}
Transfer::~Transfer()
{

}
void 
Transfer::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Siyi, 0);
    if (_uart != nullptr) {
        _initialised = true;
    }
    return; 
}

void 
Transfer::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    //reading incoming packets from gimbal
    read_incoming_packets();

    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_req_current_angle_rad_ms) >= 500) {
        _last_req_current_angle_rad_ms = now_ms;
        if (!got_gimbal_focus) {
            request_gimbal_focus();
            return;
        } else {
            request_gimbal_attitude();
        }
    } 

    if(gcs_photo_cmd)
    {
        send_1byte_packet(siyicommandid::PHOTO, 0);
        gcs_photo_cmd = false;
    }

    if(fabsf(_target_yaw) >= 0.0f && fabsf(_target_pitch) >= 0.0f && wp_acquire_cmd)
    {
        angle_rad.yaw = _target_yaw;
        angle_rad.pitch = _target_pitch;
        const int16_t yaw_and_pitch[] {(int16_t)angle_rad.yaw, (int16_t)angle_rad.pitch};
        send_packet(siyicommandid::SET_GIMBAL_ATTITUDE, yaw_and_pitch, sizeof(yaw_and_pitch));
        wp_acquire_cmd = false;
    }

    if(fabsf(_gimbal_focus) >= 0.0f && mavset_gimbal_focus)
    {
        float focus = _gimbal_focus * 0.1f;
        uint8_t absolute_movement_int = uint8_t(focus);
        uint8_t absolute_movement_float = (focus - absolute_movement_int)*10;
        const uint8_t auto_zoom_focus[] {absolute_movement_int, absolute_movement_float};
        send_packet_focus(siyicommandid::AUTO_ZOOM_FOCUS, auto_zoom_focus, sizeof(auto_zoom_focus));
        mavset_gimbal_focus = false;
        //gcs().send_text(MAV_SEVERITY_INFO, "int=%d, float=%d", absolute_movement_int, absolute_movement_float);
    }

    if(fabs(gcs_target_yaw) >= 0.0f && fabs(gcs_target_pitch) >= 0.0f && gcs_set_cmd)
    {
        angle_rad.yaw = gcs_target_yaw*10.0f;
        angle_rad.pitch = gcs_target_pitch*10.0f;
        const int16_t yaw_and_pitch[] {(int16_t)angle_rad.yaw, (int16_t)angle_rad.pitch};
        send_packet(siyicommandid::SET_GIMBAL_ATTITUDE, yaw_and_pitch, sizeof(yaw_and_pitch));
        gcs_set_cmd = false;
    }

    if(fabsf(gcs_gimbal_focus) >= 0.0f && gcs_gimbal_foc)
    {
        float focus = gcs_gimbal_focus * 0.1f;
        //gcs().send_text(MAV_SEVERITY_INFO, "focus=%f", focus);
        uint8_t absolute_movement_int = uint8_t(focus);
        uint8_t absolute_movement_float = (focus - absolute_movement_int)*10;
        const uint8_t auto_zoom_focus[] {absolute_movement_int, absolute_movement_float};
        send_packet_focus(siyicommandid::AUTO_ZOOM_FOCUS, auto_zoom_focus, sizeof(auto_zoom_focus));
        gcs_gimbal_foc = false;
        //gcs().send_text(MAV_SEVERITY_INFO, "movement_int=%d, movement_float=%d", absolute_movement_int, absolute_movement_float);
    }
}
// reading incoming packets from gimbal and confirm they are of the correct format
// results are held in the _parsed_msg structure
void 
Transfer::read_incoming_packets()
{
     // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);
    if (nbytes <= 0 ) {
        return;
    }

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t b = _uart->read();

        // sanity check byte
        if ((b < 0) || (b > 0xFF)) {
            continue;
        }

        _msg_buff[_msg_buff_len++] = b;

        // protect against overly long messages
        if (_msg_buff_len >= AP_MOUNT_SIYI_PACKETLEN_MAX) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER_LOW:
            if (b == AP_MOUNT_SIYI_HEADER1) {
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER_HIGH;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER_HIGH:
            if (b == AP_MOUNT_SIYI_HEADER2) {
                _parsed_msg.state = ParseState::WAITING_FOR_CTRL;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_CTRL:
            _parsed_msg.state = ParseState::WAITING_FOR_DATALEN_LOW;
            break;

        case ParseState::WAITING_FOR_DATALEN_LOW:
            _parsed_msg.data_len = b;
            _parsed_msg.state = ParseState::WAITING_FOR_DATALEN_HIGH;
            break;

        case ParseState::WAITING_FOR_DATALEN_HIGH:
            _parsed_msg.data_len |= ((uint16_t)b << 8);
            // sanity check data length
            if (_parsed_msg.data_len <= AP_MOUNT_SIYI_DATALEN_MAX) {
                _parsed_msg.state = ParseState::WAITING_FOR_SEQ_LOW;
            } else {
                reset_parser = true;
                //debug("data len too large:%u (>%u)", (unsigned)_parsed_msg.data_len, (unsigned)AP_MOUNT_SIYI_DATALEN_MAX);
            }
            break;

        case ParseState::WAITING_FOR_SEQ_LOW:
            _parsed_msg.state = ParseState::WAITING_FOR_SEQ_HIGH;
            break;

        case ParseState::WAITING_FOR_SEQ_HIGH:
            _parsed_msg.state = ParseState::WAITING_FOR_CMDID;
            break;

        case ParseState::WAITING_FOR_CMDID:
            _parsed_msg.command_id = b;
            _parsed_msg.data_bytes_received = 0;
            if (_parsed_msg.data_len > 0) {
                _parsed_msg.state = ParseState::WAITING_FOR_DATA;
            } else {
                _parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
            }
            break;

        case ParseState::WAITING_FOR_DATA:
            _parsed_msg.data_bytes_received++;
            if (_parsed_msg.data_bytes_received >= _parsed_msg.data_len) {
                _parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
            }
            break;

        case ParseState::WAITING_FOR_CRC_LOW:
            _parsed_msg.crc16 = b;
            _parsed_msg.state = ParseState::WAITING_FOR_CRC_HIGH;
            break;

        case ParseState::WAITING_FOR_CRC_HIGH:
            _parsed_msg.crc16 |= ((uint16_t)b << 8);

            // check crc
            const uint16_t expected_crc = crc16_ccitt(_msg_buff, _msg_buff_len-2, 0);
            if (expected_crc == _parsed_msg.crc16) {
                // successfully received a message, do something with it
                process_packet();
            } else {
                //debug("crc expected:%x got:%x", (unsigned)expected_crc, (unsigned)_parsed_msg.crc16);
            }
            reset_parser = true;
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER_LOW;
            _msg_buff_len = 0;
        }
    }
}

// 解析数据包
void 
Transfer::process_packet()
{
    // 警告意外数据缓冲区长度的标志

    switch ((siyicommandid)_parsed_msg.command_id) {

    case siyicommandid::ACQUIRE_GIMBAL_FOCUS:
        if(_parsed_msg.data_bytes_received != 2){
            break;
        }
        gimbal_focus = (uint8_t)_msg_buff[_msg_buff_data_start+1] + (uint8_t)_msg_buff[_msg_buff_data_start]*10;
        gcs().send_text(MAV_SEVERITY_INFO, "gimbal_focus=%d", gimbal_focus);
        got_gimbal_focus = true;
        break;
    case siyicommandid::ACQUIRE_GIMBAL_ATTITUDE: {
        if (_parsed_msg.data_bytes_received != 12) {
            break;
        }
        _current_angle_z = (int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+1], _msg_buff[_msg_buff_data_start]);   // yaw angle
        _current_angle_y = (int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+3], _msg_buff[_msg_buff_data_start+2]);  // pitch angle
        _current_angle_x = (int16_t)UINT16_VALUE(_msg_buff[_msg_buff_data_start+5], _msg_buff[_msg_buff_data_start+4]);  // roll angle
        got_gimbal_focus = false;
        break;
    }

    default:
        break;
    }
}

bool 
Transfer::send_packet(siyicommandid cmd_id, const int16_t* databuff, uint8_t databuff_len)
{
    // calculate and sanity check packet size
    const uint16_t packet_size = AP_MOUNT_SIYI_PACKETLEN_MIN + databuff_len;
    if (packet_size > AP_MOUNT_SIYI_PACKETLEN_MAX) {
        //debug("send_packet data buff too large");
        return false;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < packet_size) {
        return false;
    }

    // buffer for holding outgoing packet
    uint8_t send_buff[packet_size];
    uint8_t send_buff_ofs = 0;

    // packet header
    send_buff[send_buff_ofs++] = AP_MOUNT_SIYI_HEADER1;
    send_buff[send_buff_ofs++] = AP_MOUNT_SIYI_HEADER2;

    // CTRL.  Always request ACK
    send_buff[send_buff_ofs++] = 1;

    // Data_len.  protocol supports uint16_t but messages are never longer than 22 bytes
    send_buff[send_buff_ofs++] = databuff_len;
    send_buff[send_buff_ofs++] = 0;

    // SEQ (sequence)
    send_buff[send_buff_ofs++] = LOWBYTE(_last_seq);
    send_buff[send_buff_ofs++] = HIGHBYTE(_last_seq++);

    // CMD_ID
    send_buff[send_buff_ofs++] = (uint8_t)cmd_id;

    // DATA
    if (databuff_len != 0) {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    // CRC16
    const uint16_t crc = crc16_ccitt(send_buff, send_buff_ofs, 0);
    send_buff[send_buff_ofs++] = LOWBYTE(crc);
    send_buff[send_buff_ofs++] = HIGHBYTE(crc);

    // send packet
    _uart->write(send_buff, send_buff_ofs);
    return true;
}

bool 
Transfer::send_packet_focus(siyicommandid cmd_id, const uint8_t* databuff, uint8_t databuff_len)
{
    const uint16_t packet_size = AP_MOUNT_SIYI_PACKETLEN_MIN + databuff_len;
    if (packet_size > AP_MOUNT_SIYI_PACKETLEN_MAX) {
        return false;
    }

    if (_uart->txspace() < packet_size) {
        return false;
    }

    uint8_t send_buff[packet_size];
    uint8_t send_buff_ofs = 0;

    send_buff[send_buff_ofs++] = AP_MOUNT_SIYI_HEADER1;
    send_buff[send_buff_ofs++] = AP_MOUNT_SIYI_HEADER2;

    send_buff[send_buff_ofs++] = 1;

    send_buff[send_buff_ofs++] = databuff_len;
    send_buff[send_buff_ofs++] = 0;

    send_buff[send_buff_ofs++] = 0;
    send_buff[send_buff_ofs++] = 0;

    // CMD_ID
    send_buff[send_buff_ofs++] = (uint8_t)cmd_id;

    // DATA
    if (databuff_len != 0) {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }
 
    // CRC16
    const uint16_t crc = crc16_ccitt(send_buff, send_buff_ofs, 0);
    send_buff[send_buff_ofs++] = LOWBYTE(crc);
    send_buff[send_buff_ofs++] = HIGHBYTE(crc);

    // send packet
    _uart->write(send_buff, send_buff_ofs);

    return true;
}

bool 
Transfer::setCopterAltitudeToMavlink(uint8_t mav_id,mavlink_message_t* mav_link_msg)
{
    mavlink_data96_t data_96={0};
    data_96.type=0;
    data_96.len=8;
    memcpy(&data_96.data[0],&_current_angle_z,2);
    memcpy(&data_96.data[2],&_current_angle_y,2);
    memcpy(&data_96.data[4],&_current_angle_x,2);
    memcpy(&data_96.data[6],&gimbal_focus,2);
    mavlink_msg_data96_encode(mav_id,1,mav_link_msg,&data_96);
    return true;
}

void
Transfer::get_target_altitude_input(int16_t target_yaw, int16_t target_pitch, int16_t mav_gimbal_focus)
{
    _target_yaw = target_yaw;
    _target_pitch = target_pitch;
    _gimbal_focus = mav_gimbal_focus;
    wp_acquire_cmd = true;
    mavset_gimbal_focus = true;
    //gcs().send_text(MAV_SEVERITY_INFO, "yaw_angle=%d, pitch_angle=%d, gimbal_focus=%d", _target_yaw, _target_pitch, _gimbal_focus);
}
void 
Transfer::handle_to_transfer(const mavlink_message_t &msg)
{
    mavlink_data96_t pack;
    mavlink_msg_data96_decode(&msg,&pack);

    //地面站设置云台角度
    if(pack.type==0 && pack.len==6){
      gcs_target_yaw = (int16_t)UINT16_VALUE(pack.data[1], pack.data[0]);
      gcs_target_pitch = (int16_t)UINT16_VALUE(pack.data[3], pack.data[2]);
      gcs_gimbal_focus = (int16_t)UINT16_VALUE(pack.data[5], pack.data[4]);
      gcs_set_cmd = true;
      gcs_gimbal_foc = true;
    }
}

 // send a packet with a single data byte to gimbal
// returns true on success, false if outgoing serial buffer is full
bool 
Transfer::send_1byte_packet(siyicommandid cmd_id, uint8_t data_byte)
{
    return send_packet_focus(cmd_id, &data_byte, 1);
}

void
Transfer::take_photo(bool photo)
{
    gcs_photo_cmd = photo;
}