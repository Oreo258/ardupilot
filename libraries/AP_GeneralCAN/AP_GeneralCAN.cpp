/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

#include <AP_Proximity/AP_Proximity.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_GeneralCAN.h"
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "GeneralCAN",  fmt, #args); } while (0)

AP_GeneralCAN::AP_GeneralCAN()
{
    debug_can(AP_CANManager::LOG_INFO, "GeneralCAN: constructed\n\r");
}

AP_GeneralCAN *AP_GeneralCAN::get_gcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_GeneralCAN) {
        return nullptr;
    }
    return static_cast<AP_GeneralCAN*>(AP::can().get_driver(driver_index));
}


bool AP_GeneralCAN::add_interface(AP_HAL::CANIface* can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "GeneralCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "GeneralCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "GeneralCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "GeneralCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}


// initialise GeneralCAN bus
void AP_GeneralCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "GeneralCAN: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "GeneralCAN: already initialized\n\r");
        return;
    }

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "GeneralCAN: Interface not found\n\r");
        return;
    }

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_GeneralCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "GeneralCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "GeneralCAN: init done\n\r");

    return;
}

// loop to send output to ESCs in background thread
void AP_GeneralCAN::loop()
{
    AP_HAL::CANFrame rxFrame;

    while (true) {
        if (!_initialized) {
            // if not initialised wait 2ms
            debug_can(AP_CANManager::LOG_DEBUG, "GeneralCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }

        uint64_t timeout = AP_HAL::micros64() + 250ULL;

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // Look for any message responses on the CAN bus
        while (read_frame(rxFrame, timeout)) {
            printf("read frame id %d\n", rxFrame.id);
            {
                WITH_SEMAPHORE(_sem);
                memcpy(buffer, rxFrame.data, AP_HAL::CANFrame::MaxDataLen);
                frame_id = rxFrame.id;
                _updated = true;

                static uint8_t obj_count = 0;

                if (frame_id == 0x60a) {
                    obj_count = buffer[0];
                    printf("obj count %d\n", obj_count);
                }

                if (frame_id == 0x60b) {
                    const float long_dist = (buffer[1] * 32 + (buffer[2] >> 3)) * 0.2 - 500;
                    const float lat_dist = ((buffer[2] & 0x07) * 256 + buffer[3]) * 0.2 - 204.6;
                    const float angle_deg = degrees(atan2f(lat_dist, long_dist));
                    const float distance_m = sqrtf(long_dist * long_dist + lat_dist * lat_dist);
                    printf("long_dist %.2f lat_dist %.2f angle_deg %.2f distance_m %.2f\n",
                            long_dist, lat_dist, angle_deg, distance_m);

                    AP_Proximity *proximity = AP::proximity();
                    if (proximity != nullptr) {
                        proximity->update_sector(angle_deg, distance_m, obj_count);
                    }
                }
            }
        }
    }
}

bool AP_GeneralCAN::update(uint8_t *buf, uint32_t &id)
{
    WITH_SEMAPHORE(_sem);
    if (_updated) {
        memcpy(buf, buffer, AP_HAL::CANFrame::MaxDataLen);
        id = frame_id;
        _updated = false;
        return true;
    }

    return false;
}

bool AP_GeneralCAN::write_frame(uint32_t id, uint8_t buf[], uint8_t len)
{
    AP_HAL::CANFrame txFrame;
    txFrame = {id, buf, len};

    uint64_t timeout = AP_HAL::micros64() + 1000ULL;

    return write_frame(txFrame, timeout);
}

// write frame on CAN bus
bool AP_GeneralCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout)
{
    // wait for space in buffer to send command

    bool read_select = false;
    bool write_select = true;
    bool ret;
    do {
        ret = _can_iface->select(read_select, write_select, &out_frame, timeout);
        if (!ret || !write_select) {
            // delay if no space is available to send
            hal.scheduler->delay_microseconds(50);
        }
    } while (!ret || !write_select);

    // send frame and return success
    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on success
bool AP_GeneralCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout)
{
    // wait for space in buffer to read
    bool read_select = true;
    bool write_select = false;
    int ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret || !read_select) {
        // return false if no data is available to read
        return false;
    }
    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    // read frame and return success
    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

#endif // HAL_NUM_CAN_IFACES