#pragma once
#include "PCU/Data/Data.hpp"

using Direction = EncoderSensor<Sensors_data::encoder_samples>::Direction;
using SpeetecSensor = EncoderSensor<Sensors_data::encoder_samples>;

class Speetec{
    public:
    struct Data_speetec
    {
        public:
        Direction direction_encoder;
        double speed_encoder{};
        double speed_km_h_encoder{};
        double acceleration_encoder{};
        double position_encoder{};
    };
    D1_NC inline static Data_speetec data_speetec{Direction::FORWARD,0.0f,0.0f,0.0f,0.0f};

    private:
        inline static SpeetecSensor sensor_speetec{Pinout::Speetec_A,
                                        Pinout::Speetec_B,
                                        Sensors_data::encoder_counter_distance_m,
                                        Sensors_data::encoder_sample_time_s,
                                        &data_speetec.direction_encoder,
                                        &data_speetec.position_encoder,
                                        &data_speetec.speed_encoder,
                                        &data_speetec.acceleration_encoder};

    public:
    
    Speetec()=default;

    static void start()
    {
        sensor_speetec.turn_on();
    }


    static void read()
    {
        sensor_speetec.read();
        data_speetec.speed_km_h_encoder = data_speetec.speed_encoder * 3.6;
    }
};