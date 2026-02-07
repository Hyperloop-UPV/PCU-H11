#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"


using SpeetecDirection = ST_LIB::EncoderSensor<ST_LIB::Encoder<Pinout::tim_encoder_decl>, Sensors_data::encoder_samples>::Direction;
class Speetec{
    public:

    inline static double acceleration_encoder{0.0f};
    [[maybe_unused]] inline static SpeetecDirection internal_direction{SpeetecDirection::FORWARD};

    private:

        static ST_LIB::EncoderSensor<ST_LIB::Encoder<Pinout::tim_encoder_decl>,Sensors_data::encoder_samples>* sensor_speetec;
    public:
    
    Speetec()=default;
    
    static void init(ST_LIB::Encoder<Pinout::tim_encoder_decl>* encoder_ptr);
    
    static void read();
    static bool is_going_backwards();
};