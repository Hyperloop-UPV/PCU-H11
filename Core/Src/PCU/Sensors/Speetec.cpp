#include "PCU/Sensors/Speetec.hpp"
#include "PCU/PCU.hpp"

ST_LIB::EncoderSensor<Speetec::EncoderType, Speetec::SAMPLES>* Speetec::sensor = nullptr;
double* Speetec::position_ptr = nullptr;

void Speetec::init(EncoderType* enc_ptr)
{
    position_ptr = &PCU::control_data.position_encoder;

    sensor = new ST_LIB::EncoderSensor<EncoderType, SAMPLES>(
        *enc_ptr,
        Sensors_data::encoder_counter_distance_m,
        Sensors_data::encoder_sample_time_s,
        &internal_direction,
        position_ptr,
        &speed_encoder,
        &acceleration_encoder
    );

    sensor->turn_on();
}

void Speetec::read()
{
    if(sensor == nullptr) return;

    sensor->read();

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