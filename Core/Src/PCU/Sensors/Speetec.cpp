#include "PCU/Sensors/Speetec.hpp"
#include "PCU/PCU.hpp"

ST_LIB::EncoderSensor<ST_LIB::Encoder<Pinout::tim_encoder_decl>, Sensors_data::encoder_samples>* Speetec::sensor_speetec = nullptr;

void Speetec::init(ST_LIB::Encoder<Pinout::tim_encoder_decl>* encoder_ptr)
{
    sensor_speetec = new ST_LIB::EncoderSensor<ST_LIB::Encoder<Pinout::tim_encoder_decl>, Sensors_data::encoder_samples>(
            *encoder_ptr,
            Sensors_data::encoder_counter_distance_m,
            Sensors_data::encoder_sample_time_s,
            &internal_direction,
            &PCU::control_data.position_encoder,
            &PCU::control_data.speed_encoder,
            &acceleration_encoder
        );

    sensor_speetec->turn_on();

}

void Speetec::read()
{
    if(sensor_speetec) sensor_speetec->read();
    if(is_going_backwards())
        PCU::control_data.established_direction = EncoderDirection::Backward;
    else
        PCU::control_data.established_direction = EncoderDirection::Forward;
    PCU::control_data.speed_km_h_encoder = PCU::control_data.speed_encoder * 3.6; 


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