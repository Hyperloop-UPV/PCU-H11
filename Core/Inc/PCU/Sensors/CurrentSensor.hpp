#pragma once
#include "PCU/Data/Data.hpp"



class CurrentSensors{
    public: 

    struct Data_current
    {
        public:
        float actual_current_sensor_u_a;
        float actual_current_sensor_u_b;
        float actual_current_sensor_v_a;
        float actual_current_sensor_v_b;
        float actual_current_sensor_w_a;
        float actual_current_sensor_w_b;
    };

    D1_NC inline static Data_current data_current{0,0,0,0,0,0};

    private:
        inline static LinearSensor<float> sensor_u_a{Pinout::Current_sensor_U_A,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&data_current.actual_current_sensor_u_a};
        inline static LinearSensor<float> sensor_u_b{Pinout::Current_sensor_U_B,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&data_current.actual_current_sensor_u_b};
        inline static LinearSensor<float> sensor_v_a{Pinout::Current_sensor_V_A,Sensors_data::slope_current_sensor_inverted,Sensors_data::offset_current_sensor_inverted,&data_current.actual_current_sensor_v_a};
        inline static LinearSensor<float> sensor_v_b{Pinout::Current_sensor_V_B,Sensors_data::slope_current_sensor_inverted,Sensors_data::offset_current_sensor_inverted,&data_current.actual_current_sensor_v_b};
        inline static LinearSensor<float> sensor_w_a{Pinout::Current_sensor_W_A,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&data_current.actual_current_sensor_w_a};
        inline static LinearSensor<float> sensor_w_b{Pinout::Current_sensor_W_B,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&data_current.actual_current_sensor_w_b};
        
    public:

    CurrentSensors()=default;
    static void read();
    static void zeroing();

};