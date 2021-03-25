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
#include <AP_GeneralCAN/AP_GeneralCAN.h>
#include "AP_Proximity_SR73F.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAV_TIMEOUT_MS    500 // distance messages must arrive within this many milliseconds

AP_Proximity_SR73F::AP_Proximity_SR73F(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state):
    AP_Proximity_Backend(_frontend, _state)
{

}

// update the state of the sensor
void AP_Proximity_SR73F::update(void)
{
    // check for timeout and set health status
    if (_last_update_ms == 0 || (AP_HAL::millis() - _last_update_ms > PROXIMITY_MAV_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

void AP_Proximity_SR73F::update_sector(const float angle, const float dist, const uint8_t object_count)
{
    if (!_initialised) {
        _initialised = initialise();
        return;
    }

    static uint8_t obj_count = 0;

   const uint8_t sector = convert_angle_to_sector(angle);

    // min
    if (_distance_min_last[sector] > dist) {
        _distance_min_last[sector] = dist;
        _angle_last[sector] = angle;
        _distance_valid_last[sector] = true;
    }

    if (++obj_count != object_count) {
        return;
    }

    obj_count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        _angle[i] = _angle_last[i];
        _distance[i] = _distance_min_last[i];
        _distance_valid[i] = _distance_valid_last[i];
        update_boundary_for_sector(i, true);

        _distance_min_last[i] = MAXFLOAT;
        _distance_valid_last[i] = false;
    }

    _last_update_ms = AP_HAL::millis();
}

bool AP_Proximity_SR73F::initialise()
{
    return true;
}

// get maximum distance (in meters) of sensor
float AP_Proximity_SR73F::distance_max() const
{
    return 40.0f;
}

// get minimum distance (in meters) of sensor
float AP_Proximity_SR73F::distance_min() const
{
    return 0.20f;
}