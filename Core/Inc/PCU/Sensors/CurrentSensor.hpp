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
    // static ST_LIB::DigitalOutputDomain::Instance& hall_supply_a;
    // static ST_LIB::DigitalOutputDomain::Instance& hall_supply_b;
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
    #if PCU_H10 == 0
    // static void init(ST_LIB::DigitalOutputDomain::Instance& hall_supply_a_instance,
    //                  ST_LIB::DigitalOutputDomain::Instance& hall_supply_b_instance)
    // {
    //     hall_supply_a = hall_supply_a_instance;
    //     hall_supply_b = hall_supply_b_instance;
    // }
    static void start()
    {
        // hall_supply_a.turn_on();
        // hall_supply_b.turn_on();
        is_on = true;
    }

    static void stop()
    {
        // hall_supply_a.turn_off();
        // hall_supply_b.turn_off();
        is_on = false;
    }
    #endif
    static void read();
    static void zeroing();

};