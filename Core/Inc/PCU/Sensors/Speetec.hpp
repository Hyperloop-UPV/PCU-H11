#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"

enum class SpeetecDirection : uint8_t { FORWARD = 0, BACKWARDS = 1 };

class Speetec{
    public:

    inline static double acceleration_encoder{0.0f};
    [[maybe_unused]] inline static SpeetecDirection internal_direction{SpeetecDirection::FORWARD};
    inline static double speed_encoder{0.0f};

    private:

        static ST_LIB::Encoder<Pinout::tim_encoder_decl>* encoder_ptr;
        
        static constexpr size_t SAMPLES = Sensors_data::encoder_samples;
        static constexpr int64_t START_COUNTER = UINT32_MAX / 2;
        
        static RingBuffer<int64_t, (SAMPLES / 2) * 2> past_delta_counters;
        static double* position_ptr;
    public:
    
    Speetec()=default;
    
    static void init(ST_LIB::Encoder<Pinout::tim_encoder_decl>* enc_ptr);
    
    static void read();
    static bool is_going_backwards();
};