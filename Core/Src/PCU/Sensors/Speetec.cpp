#include "PCU/Sensors/Speetec.hpp"
#include "PCU/PCU.hpp"

SpeetecSensor Speetec::sensor_speetec(
    Pinout::Speetec_A,
    Pinout::Speetec_B,
    Sensors_data::encoder_counter_distance_m,
    Sensors_data::encoder_sample_time_s,
    &internal_direction,
    &position_encoder,
    &speed_encoder,
    &acceleration_encoder);

void Speetec::read()
{
    sensor_speetec.read();
    PCU::control_data.speed_km_h_encoder = speed_encoder * 3.6;
    if(internal_direction == SpeetecDirection::FORWARD)
        PCU::control_data.established_direction = EncoderDirection::Forward;
    else
        PCU::control_data.established_direction = EncoderDirection::Backward;
}