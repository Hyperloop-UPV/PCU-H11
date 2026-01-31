#pragma once
#include "PCU/Data/Data.hpp"
#include "PCU/PCU.hpp"

using Direction = EncoderSensor<Sensors_data::encoder_samples>::Direction;
using SpeetecSensor = EncoderSensor<Sensors_data::encoder_samples>;

class Speetec{
    public:

    inline static D1_NC double speed_encoder{0.0f};
    inline static D1_NC double speed_km_h_encoder{0.0f};
    inline static D1_NC double acceleration_encoder{0.0f};
    inline static D1_NC double position_encoder{0.0f};
    private:
        inline static SpeetecSensor sensor_speetec{Pinout::Speetec_A,
                                        Pinout::Speetec_B,
                                        Sensors_data::encoder_counter_distance_m,
                                        Sensors_data::encoder_sample_time_s,
                                        &PCU::control_data.Stablished_direction,
                                        &position_encoder,
                                        &speed_encoder,
                                        &acceleration_encoder};

    public:
    
    Speetec()=default;
    
    static void init()
    {
        sensor_speetec.turn_on();
    }

    
    static void read()
    {
        sensor_speetec.read();
        speed_km_h_encoder = speed_encoder * 3.6;
    }
};