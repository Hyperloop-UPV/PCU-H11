#include "PCU/Sensors/CurrentSensor.hpp"

#define TIMES_TO_CREATE_ZERO 1000

void CurrentSensors::init(ST_LIB::ADCDomain::Instance& Sensor_U_A,ST_LIB::ADCDomain::Instance& Sensor_U_B,ST_LIB::ADCDomain::Instance& Sensor_V_A,
    ST_LIB::ADCDomain::Instance& Sensor_V_B,ST_LIB::ADCDomain::Instance& Sensor_W_A,ST_LIB::ADCDomain::Instance& Sensor_W_B)
{
    sensor_u_a = new LinearSensor<float>{Sensor_U_A,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&actual_current_sensor_u_a};
    sensor_u_b = new LinearSensor<float>{Sensor_U_B,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&actual_current_sensor_u_b};
    sensor_v_a = new LinearSensor<float>{Sensor_V_A,Sensors_data::slope_current_sensor_inverted,Sensors_data::offset_current_sensor_inverted,&actual_current_sensor_v_a};
    sensor_v_b = new LinearSensor<float>{Sensor_V_B,Sensors_data::slope_current_sensor_inverted,Sensors_data::offset_current_sensor_inverted,&actual_current_sensor_v_b};
    sensor_w_a = new LinearSensor<float>{Sensor_W_A,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&actual_current_sensor_w_a};
    sensor_w_b = new LinearSensor<float>{Sensor_W_B,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&actual_current_sensor_w_b};
}

void CurrentSensors::read()
{
    sensor_u_a->read();
    sensor_u_b->read();
    sensor_v_a->read();
    sensor_v_b->read();
    sensor_w_a->read();
    sensor_w_b->read();
}

void CurrentSensors::zeroing()
{
    float new_offset_u_a = 0;
    float new_offset_u_b = 0;
    float new_offset_v_a = 0;
    float new_offset_v_b = 0;
    float new_offset_w_a = 0;
    float new_offset_w_b = 0;
    for(size_t i = 1; i < TIMES_TO_CREATE_ZERO; i++)
    {
        read();
        new_offset_u_a = (new_offset_u_a * (i - 1) + actual_current_sensor_u_a)/i;
        new_offset_u_b = (new_offset_u_b * (i - 1) + actual_current_sensor_u_b)/i;
        new_offset_v_a = (new_offset_v_a * (i - 1) + actual_current_sensor_v_a)/i;
        new_offset_v_b = (new_offset_v_b * (i - 1) + actual_current_sensor_v_b)/i;
        new_offset_w_a = (new_offset_w_a * (i - 1) + actual_current_sensor_w_a)/i;
        new_offset_w_b = (new_offset_w_b * (i - 1) + actual_current_sensor_w_b)/i;
    }    sensor_u_a->set_offset(sensor_u_a->get_offset()-new_offset_u_a);
    sensor_u_b->set_offset(sensor_u_b->get_offset()-new_offset_u_b);
    sensor_v_a->set_offset(sensor_v_a->get_offset()-new_offset_v_a);
    sensor_v_b->set_offset(sensor_v_b->get_offset()-new_offset_v_b);
    sensor_w_a->set_offset(sensor_w_a->get_offset()-new_offset_w_a);
    sensor_w_b->set_offset(sensor_w_b->get_offset()-new_offset_w_b);
    // Watchdog::refresh();
}