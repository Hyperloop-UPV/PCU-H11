#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"


using SpeetecDirection = ST_LIB::EncoderSensor<Pinout::encoder_timer, Sensors_data::encoder_samples>::Direction;
class Speetec{
    public:

    inline static D1_NC double speed_encoder{0.0f};
    inline static D1_NC double speed_km_h_encoder{0.0f};
    inline static D1_NC double acceleration_encoder{0.0f};
    inline static D1_NC double position_encoder{0.0f};
    inline static D1_NC SpeetecDirection internal_direction{SpeetecDirection::FORWARD};

    private:
        inline static ST_LIB::EncoderSensor<Pinout::encoder_timer,Sensors_data::encoder_samples> sensor_speetec(
            Sensors_data::encoder_counter_distance_m,
            Sensors_data::encoder_sample_time_s,
            &internal_direction,
            &position_encoder,
            &speed_encoder,
            &acceleration_encoder);

    public:
    
    Speetec()=default;
    
    static void init()
    {
        sensor_speetec.turn_on();
    }
    
    static void read();
    static bool is_going_backwards();
};