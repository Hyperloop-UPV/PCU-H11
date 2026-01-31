#pragma once
#include "PCU/Data/Data.hpp"

class CurrentSensors{
    public: 

    inline static D1_NC float actual_current_sensor_u_a{0.0f};
    inline static D1_NC float actual_current_sensor_u_b{0.0f};
    inline static D1_NC float actual_current_sensor_v_a{0.0f};
    inline static D1_NC float actual_current_sensor_v_b{0.0f};
    inline static D1_NC float actual_current_sensor_w_a{0.0f};
    inline static D1_NC float actual_current_sensor_w_b{0.0f};
    #if PCU_H10 == 0
    
    inline static bool is_on{false};
    #endif


    private:
        inline static LinearSensor<float> sensor_u_a{Pinout::Current_sensor_U_A,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&actual_current_sensor_u_a};
        inline static LinearSensor<float> sensor_u_b{Pinout::Current_sensor_U_B,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&actual_current_sensor_u_b};
        inline static LinearSensor<float> sensor_v_a{Pinout::Current_sensor_V_A,Sensors_data::slope_current_sensor_inverted,Sensors_data::offset_current_sensor_inverted,&actual_current_sensor_v_a};
        inline static LinearSensor<float> sensor_v_b{Pinout::Current_sensor_V_B,Sensors_data::slope_current_sensor_inverted,Sensors_data::offset_current_sensor_inverted,&actual_current_sensor_v_b};
        inline static LinearSensor<float> sensor_w_a{Pinout::Current_sensor_W_A,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&actual_current_sensor_w_a};
        inline static LinearSensor<float> sensor_w_b{Pinout::Current_sensor_W_B,Sensors_data::slope_current_sensor,Sensors_data::offset_current_sensor,&actual_current_sensor_w_b};
        
    public:

    CurrentSensors()=default;

    static void read();
    static void zeroing();

};