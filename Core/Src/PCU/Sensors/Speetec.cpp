#include "PCU/Sensors/Speetec.hpp"
#include "PCU/PCU.hpp"

SpeetecSensor Speetec::sensor_speetec(
    Pinout::Speetec_A,
    Pinout::Speetec_B,
    Sensors_data::encoder_counter_distance_m,
    Sensors_data::encoder_sample_time_s,
    &PCU::control_data.established_direction,
    &position_encoder,
    &speed_encoder,
    &acceleration_encoder);

