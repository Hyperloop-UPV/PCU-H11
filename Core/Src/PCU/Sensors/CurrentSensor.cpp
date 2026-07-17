#include "PCU/Sensors/CurrentSensor.hpp"

#define TIMES_TO_CREATE_ZERO 1000

void CurrentSensors::init(ST_LIB::ADCDomain::Instance& Sensor_U_A,ST_LIB::ADCDomain::Instance& Sensor_U_B,ST_LIB::ADCDomain::Instance& Sensor_V_A,
    ST_LIB::ADCDomain::Instance& Sensor_V_B,ST_LIB::ADCDomain::Instance& Sensor_W_A,ST_LIB::ADCDomain::Instance& Sensor_W_B)
{
    constexpr float hall_UA_slope = 112.23f;
    constexpr float hall_UA_offset = -189.02f;

    constexpr float hall_VA_slope = 101.96f;
    constexpr float hall_VA_offset = 171.06f;

    constexpr float hall_WA_slope = 147.37f;
    constexpr float hall_WA_offset = -248.84f;

    constexpr float hall_UB_slope = 150.97f;
    constexpr float hall_UB_offset = -258.80f;

    constexpr float hall_VB_slope = 143.70f;
    constexpr float hall_VB_offset = 242.85f;

    constexpr float hall_WB_slope = 113.53f;
    constexpr float hall_WB_offset = -191.13f;

    sensor_u_a = new LinearSensor<float>{Sensor_U_A, hall_UA_slope,
        hall_UA_offset, &actual_current_sensor_u_a};
    sensor_v_a = new LinearSensor<float>{Sensor_V_A, hall_VA_slope, 
        hall_VA_offset, &actual_current_sensor_v_a};
    sensor_w_a = new LinearSensor<float>{Sensor_W_A, hall_WA_slope, 
        hall_WA_offset,&actual_current_sensor_w_a};

    sensor_u_b = new LinearSensor<float>{Sensor_U_B, hall_UB_slope, 
        hall_UB_offset,&actual_current_sensor_u_b};
    sensor_v_b = new LinearSensor<float>{Sensor_V_B, hall_VB_slope, 
        hall_VB_offset,&actual_current_sensor_v_b};
    sensor_w_b = new LinearSensor<float>{Sensor_W_B, hall_WB_slope, 
        hall_WB_offset,&actual_current_sensor_w_b};
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

void zeroing()
{
    float new_offset_u_a = 0;
    float new_offset_u_b = 0;
    float new_offset_v_a = 0;
    float new_offset_v_b = 0;
    float new_offset_w_a = 0;
    float new_offset_w_b = 0;
    for(size_t i = 1; i < TIMES_TO_CREATE_ZERO; i++)
    {
        CurrentSensors::read();
        new_offset_u_a = (new_offset_u_a * (i - 1) + CurrentSensors::actual_current_sensor_u_a)/i;
        new_offset_u_b = (new_offset_u_b * (i - 1) + CurrentSensors::actual_current_sensor_u_b)/i;
        new_offset_v_a = (new_offset_v_a * (i - 1) + CurrentSensors::actual_current_sensor_v_a)/i;
        new_offset_v_b = (new_offset_v_b * (i - 1) + CurrentSensors::actual_current_sensor_v_b)/i;
        new_offset_w_a = (new_offset_w_a * (i - 1) + CurrentSensors::actual_current_sensor_w_a)/i;
        new_offset_w_b = (new_offset_w_b * (i - 1) + CurrentSensors::actual_current_sensor_w_b)/i;

        // This is so slow...
        // Watchdog::refresh();
    }
    CurrentSensors::sensor_u_a->set_offset(CurrentSensors::sensor_u_a->get_offset()-new_offset_u_a);
    CurrentSensors::sensor_u_b->set_offset(CurrentSensors::sensor_u_b->get_offset()-new_offset_u_b);
    CurrentSensors::sensor_v_a->set_offset(CurrentSensors::sensor_v_a->get_offset()-new_offset_v_a);
    CurrentSensors::sensor_v_b->set_offset(CurrentSensors::sensor_v_b->get_offset()-new_offset_v_b);
    CurrentSensors::sensor_w_a->set_offset(CurrentSensors::sensor_w_a->get_offset()-new_offset_w_a);
    CurrentSensors::sensor_w_b->set_offset(CurrentSensors::sensor_w_b->get_offset()-new_offset_w_b);
    // Watchdog::refresh();
}