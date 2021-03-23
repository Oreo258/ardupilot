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

#pragma once

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>

class AP_GeneralCAN : public AP_CANDriver {
public:
    AP_GeneralCAN();
    ~AP_GeneralCAN();

    /* Do not allow copies */
    AP_GeneralCAN(const AP_GeneralCAN &other) = delete;
    AP_GeneralCAN &operator=(const AP_GeneralCAN&) = delete;

    // Return ToshibaCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_GeneralCAN *get_gcan(uint8_t driver_index);

    // initialise ToshibaCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from where?
    bool update(uint8_t *buf, uint32_t &id);

    bool write_frame(uint32_t id, uint8_t buf[], uint8_t len);

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on success
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    bool _initialized;
    char _thread_name[9];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;

    // telemetry data
    HAL_Semaphore _sem;
    bool _updated;
    uint8_t buffer[8];
    uint32_t frame_id;
};