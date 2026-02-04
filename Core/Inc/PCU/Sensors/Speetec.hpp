#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"


using SpeetecDirection = ST_LIB::EncoderSensor<Pinout::encoder_timer, Sensors_data::encoder_samples>::Direction;
class Speetec{
    public:

    inline static double speed_encoder{0.0f};
    inline static double speed_km_h_encoder{0.0f};
    inline static double acceleration_encoder{0.0f};
    inline static double position_encoder{0.0f};
    [[maybe-unused]] inline static SpeetecDirection internal_direction{SpeetecDirection::FORWARD};

    private:
        inline static ST_LIB::Encoder<Pinout::encoder_timer>* encoder_instance{};

        static ST_LIB::EncoderSensor<Pinout::encoder_timer,Sensors_data::encoder_samples> sensor_speetec;
    public:
    
    Speetec()=default;
    
    static void init(ST_LIB::Encoder<Pinout::encoder_timer>* encoder_ptr);
    
    static void read();
    static bool is_going_backwards();
};