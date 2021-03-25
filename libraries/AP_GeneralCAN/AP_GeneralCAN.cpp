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

#if HAL_WITH_UAVCAN
#include <AP_Proximity/AP_Proximity.h>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_GeneralCAN.h"
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>

static const uint8_t CAN_IFACE_INDEX = 0;

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

AP_GeneralCAN::AP_GeneralCAN()
{
    debug_can(2, "GeneralCAN: constructed\n\r");
}

AP_GeneralCAN *AP_GeneralCAN::get_gcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_GeneralCAN) {
        return nullptr;
    }
    return static_cast<AP_GeneralCAN*>(AP::can().get_driver(driver_index));
}

// initialise GeneralCAN bus
void AP_GeneralCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(2, "GeneralCAN: starting init\n\r");

    if (_initialized) {
        debug_can(1, "GeneralCAN: already initialized\n\r");
        return;
    }

    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr) {
        debug_can(1, "GeneralCAN: no mgr for this driver\n\r");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_can(1, "GeneralCAN: mgr not initialized\n\r");
        return;
    }

    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can(1, "GeneralCAN: no CAN driver\n\r");
        return;
    }

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_GeneralCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(1, "GeneralCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(2, "GeneralCAN: init done\n\r");

    return;
}

// loop to send output to ESCs in background thread
void AP_GeneralCAN::loop()
{
    uavcan::CanFrame rxFrame;
    uavcan::MonotonicTime timeout;
    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), 500);

    while (true) {
        if (!_initialized) {
            // if not initialised wait 2ms
            debug_can(2, "GeneralCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }

        timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + timeout_us);

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // Look for any message responses on the CAN bus
        while (read_frame(rxFrame, timeout)) {
            {
                WITH_SEMAPHORE(_sem);
                memcpy(buffer, rxFrame.data, 8);
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
        memcpy(buf, buffer, 8);
        id = frame_id;
        _updated = false;
        return true;
    }

    return false;
}

bool AP_GeneralCAN::write_frame(uint32_t id, uint8_t buf[], uint8_t len)
{
    uavcan::CanFrame txFrame;
    txFrame = {id, buf, len};

    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), 500);
    uavcan::MonotonicTime timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + timeout_us);

    return write_frame(txFrame, timeout);
}

// write frame on CAN bus
bool AP_GeneralCAN::write_frame(uavcan::CanFrame &out_frame, uavcan::MonotonicTime timeout)
{
    // wait for space in buffer to send command
    uavcan::CanSelectMasks inout_mask;
    do {
        inout_mask.read = 0;
        inout_mask.write = 1 << CAN_IFACE_INDEX;
        _select_frames[CAN_IFACE_INDEX] = &out_frame;
        _can_driver->select(inout_mask, _select_frames, timeout);

        // delay if no space is available to send
        if (!inout_mask.write) {
            hal.scheduler->delay_microseconds(50);
        }
    } while (!inout_mask.write);

    // send frame and return success
    return (_can_driver->getIface(CAN_IFACE_INDEX)->send(out_frame, timeout, uavcan::CanIOFlagAbortOnError) == 1);
}

// read frame on CAN bus, returns true on success
bool AP_GeneralCAN::read_frame(uavcan::CanFrame &recv_frame, uavcan::MonotonicTime timeout)
{
    // wait for space in buffer to read
    uavcan::CanSelectMasks inout_mask;
    inout_mask.read = 1 << CAN_IFACE_INDEX;
    inout_mask.write = 0;
    _select_frames[CAN_IFACE_INDEX] = &recv_frame;
    _can_driver->select(inout_mask, _select_frames, timeout);

    // return false if no data is available to read
    if (!inout_mask.read) {
        return false;
    }
    uavcan::MonotonicTime time;
    uavcan::UtcTime utc_time;
    uavcan::CanIOFlags flags {};

    // read frame and return success
    return (_can_driver->getIface(CAN_IFACE_INDEX)->receive(recv_frame, time, utc_time, flags) == 1);
}

#endif // HAL_NUM_CAN_IFACES