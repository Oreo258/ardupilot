#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

class AP_Proximity_SR73F : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_SR73F(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    void update_sector(const float angle, const float dist, const uint8_t object_count) override;

private:
    bool initialise();
     // horizontal distance support
    bool _initialised;
    uint32_t _last_update_ms;   // system time of last DISTANCE_SENSOR message received

    float _angle_deg_last;
    float _distance_m_last;
    uint8_t _last_sector;                   ///< last sector requested
    float _angle_last[PROXIMITY_NUM_SECTORS];
    float _distance_min_last[PROXIMITY_NUM_SECTORS];
    bool _distance_valid_last[PROXIMITY_NUM_SECTORS];
};