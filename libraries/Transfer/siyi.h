#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_SIYI_PACKETLEN_MAX     22  // maximum number of bytes in a packet sent to or received from the gimbal
class Transfer
{
public:
    Transfer();
    ~Transfer();
    static Transfer *get_singleton() { return _singleton; }
    void init();
    void update();
    //void get_mavlink_data96_input(uint8_t &mav_data){_mav_data0 = mav_data;}
    //void get_target_altitude_input(const float &target_yaw, const float &target_pitch){_target_yaw = target_yaw, _target_pitch = target_pitch;}
    void get_target_altitude_input(int16_t target_yaw, int16_t target_pitch, int16_t mav_gimbal_focus);
    void take_photo(bool photo);
    bool setCopterAltitudeToMavlink(uint8_t mav_id,mavlink_message_t* mav_link_msg);
    void handle_to_transfer(const mavlink_message_t &msg);
private:
    static Transfer *_singleton;
    AP_HAL::UARTDriver *_port = nullptr;
    // serial protocol command ids
    enum class siyicommandid {
        ACQUIRE_GIMBAL_FOCUS = 0x18,
        PHOTO = 0x0C,
        ACQUIRE_GIMBAL_ATTITUDE = 0x0D,
        SET_GIMBAL_ATTITUDE = 0x0E,
        AUTO_ZOOM_FOCUS = 0x0F
    };
    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER_LOW,
        WAITING_FOR_HEADER_HIGH,
        WAITING_FOR_CTRL,
        WAITING_FOR_DATALEN_LOW,
        WAITING_FOR_DATALEN_HIGH,
        WAITING_FOR_SEQ_LOW,
        WAITING_FOR_SEQ_HIGH,
        WAITING_FOR_CMDID,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH,
    };

    // reading incoming packets from gimbal and confirm they are of the correct format
    // results are held in the _parsed_msg structure
    void read_incoming_packets();

    // process successfully decoded packets held in the _parsed_msg structure
    void process_packet();

    // send packet to gimbal
    // returns true on success, false if outgoing serial buffer is full
    bool send_packet(siyicommandid cmd_id, const int16_t* databuff, uint8_t databuff_len);

    bool send_packet_focus(siyicommandid cmd_id, const uint8_t* databuff, uint8_t databuff_len);

    void request_gimbal_attitude() { send_packet(siyicommandid::ACQUIRE_GIMBAL_ATTITUDE, nullptr, 0);}

    void request_gimbal_focus() { send_packet(siyicommandid::ACQUIRE_GIMBAL_FOCUS, nullptr, 0);}

    // send target pitch and yaw angles to gimbal
    // yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef);

    bool send_1byte_packet(siyicommandid cmd_id, uint8_t data_byte);

    // internal variables
    AP_HAL::UARTDriver *_uart;                      // uart connected to gimbal
    bool _initialised;                              // true once the driver has been initialised

    // buffer holding bytes from latest packet.  This is only used to calculate the crc
    uint8_t _msg_buff[AP_MOUNT_SIYI_PACKETLEN_MAX];
    uint8_t _msg_buff_len;
    const uint8_t _msg_buff_data_start = 8;         // data starts at this byte of _msg_buff

    // parser state and unpacked fields
    struct PACKED {
        uint16_t data_len;                          // expected number of data bytes
        uint8_t command_id;                         // command id
        uint16_t data_bytes_received;               // number of data bytes received so far
        uint16_t crc16;                             // latest message's crc
        ParseState state;                           // state of incoming message processing
    } _parsed_msg;

    // structure for a single angle or rate target
    struct MountTarget {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        bool yaw_is_ef;
    }angle_rad;

    struct PACKED cmd_set_angles_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        float pitch;
        float roll;
        float yaw;
        uint8_t flags;
        uint8_t type;
        uint16_t crc;
    };

    // variables for sending packets to gimbal
    uint32_t _last_send_ms;                         // system time (in milliseconds) of last packet sent to gimbal
    uint16_t _last_seq;                             // last sequence number used (should be increment for each send)
    bool     _last_lock;                            // last lock value sent to gimbal
    uint8_t  _lock_send_counter;                    // counter used to resend lock status to gimbal at regular intervals

    // actual attitude received from gimbal
    //Vector3f _current_angle;                    // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    int16_t _current_angle_x;
    int16_t _current_angle_y;
    int16_t _current_angle_z;
    uint32_t _last_current_angle_rad_ms;            // system time _current_angle_rad was updated
    uint32_t _last_req_current_angle_rad_ms;        // system time that this driver last requested current angle
    int16_t gimbal_focus;                           // *1000

    // variables for camera state
    bool _last_record_video;                        // last record_video state sent to gimbal

    // mavlink data[0]
    uint8_t _mav_data0;

    //航点目标角度
    int16_t _target_yaw;
    int16_t _target_pitch;

    //变焦倍数
    int16_t _gimbal_focus;

    //地面站设置目标角度
    int16_t gcs_target_yaw;
    int16_t gcs_target_pitch;
    int16_t gcs_gimbal_focus;

    // 设置云台角度
    bool gcs_set_cmd=false;

    // 航点状态下查询当前云台角度
    bool wp_acquire_cmd =false;

    // 触发拍照
    bool gcs_photo_cmd=false;

    bool mavset_gimbal_focus=false;

    bool gcs_gimbal_foc=false;

    bool got_gimbal_focus = false;
};