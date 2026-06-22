#include "PCU/Sensors/Speetec.hpp"
#include "PCU/PCU.hpp"

ST_LIB::Encoder<Pinout::tim_encoder_decl>* Speetec::encoder_ptr = nullptr;
RingBuffer<int64_t, (Speetec::SAMPLES / 2) * 2> Speetec::past_delta_counters{};
double* Speetec::position_ptr = nullptr;

void Speetec::init(ST_LIB::Encoder<Pinout::tim_encoder_decl>* enc_ptr)
{
    encoder_ptr = enc_ptr;
    position_ptr = &PCU::control_data.position_encoder;

    // Fill ring buffer with zeros
    for (size_t i = 0; i < SAMPLES; ++i)
        past_delta_counters.push(0);

    ST_LIB::Encoder<Pinout::tim_encoder_decl>::turn_on();
}

void Speetec::read()
{
    if(encoder_ptr == nullptr) return;

    uint32_t counter = ST_LIB::Encoder<Pinout::tim_encoder_decl>::get_counter();

    int64_t delta_counter = static_cast<int64_t>(counter) - START_COUNTER;
    const int64_t& previous_delta_counter =
        past_delta_counters[past_delta_counters.size() / 2 - 1];
    const int64_t& previous_previous_delta_counter =
        past_delta_counters[past_delta_counters.size() - 1];

    *position_ptr = delta_counter * Sensors_data::encoder_counter_distance_m;

    // https://en.wikipedia.org/wiki/Finite_difference_coefficient#Backward_finite_difference
    speed_encoder = ((3.0 * delta_counter / 2.0) - (2.0 * previous_delta_counter) +
              (previous_previous_delta_counter / 2.0)) *
             Sensors_data::encoder_counter_distance_m /
             (Sensors_data::encoder_sample_time_s * past_delta_counters.size() / 2);

    acceleration_encoder =
        (delta_counter - (2.0 * previous_delta_counter) + previous_previous_delta_counter) *
        Sensors_data::encoder_counter_distance_m /
        ((Sensors_data::encoder_sample_time_s * past_delta_counters.size() / 2) *
         (Sensors_data::encoder_sample_time_s * past_delta_counters.size() / 2));

    internal_direction = ST_LIB::Encoder<Pinout::tim_encoder_decl>::get_direction()
                        ? SpeetecDirection::FORWARD : SpeetecDirection::BACKWARDS;

    past_delta_counters.push_pop(delta_counter);

    __disable_irq();
    PCU::control_data.speed_encoder = speed_encoder;
    PCU::control_data.speed_km_h_encoder = speed_encoder * 3.6; 
    __enable_irq();

}

bool Speetec::is_going_backwards()
{
    static double prev_position_encoder = 0;
    bool ret = false;
    if((PCU::control_data.position_encoder - prev_position_encoder) < 0.0)
    {
        ret = true;
    }
    prev_position_encoder = PCU::control_data.position_encoder;
    return ret;
}