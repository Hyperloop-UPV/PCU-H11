#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"

class Speetec{
    public:

    inline static double acceleration_encoder{0.0f};
    inline static ST_LIB::Direction internal_direction{ST_LIB::Direction::FORWARD};
    inline static double speed_encoder{0.0f};

    private:

        using EncoderType = ST_LIB::Encoder<Pinout::tim_encoder_decl>;
        static constexpr size_t SAMPLES = Sensors_data::encoder_samples;

        static ST_LIB::EncoderSensor<EncoderType, SAMPLES>* sensor;
        static double* position_ptr;
    public:
    
    Speetec()=default;
    
    static void init(EncoderType* enc_ptr);
    
    static void read();
    static bool is_going_backwards();
};