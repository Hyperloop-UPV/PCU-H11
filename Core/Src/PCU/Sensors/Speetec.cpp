#include "PCU/Sensors/Speetec.hpp"

void Speetec::start(){
    sensor_speetec.turn_on();
}

void Speetec::read(){
    sensor_speetec.read();
    data_speetec.speed_km_h_encoder = data_speetec.speed_encoder * 3.6;
}