#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"

using SpeetecDirection = EncoderSensor<Sensors_data::encoder_samples>::Direction;
using SpeetecSensor = EncoderSensor<Sensors_data::encoder_samples>;

class Speetec{
    public:

    inline static D1_NC double speed_encoder{0.0f};
    inline static D1_NC double speed_km_h_encoder{0.0f};
    inline static D1_NC double acceleration_encoder{0.0f};
    inline static D1_NC double position_encoder{0.0f};
    inline static D1_NC SpeetecDirection internal_direction{SpeetecDirection::FORWARD};

    private:
        static SpeetecSensor sensor_speetec;

    public:
    
    Speetec()=default;
    
    static void init()
    {
        sensor_speetec.turn_on();
    }
    
    static void read();
    static bool is_going_backwards();
};