#pragma once
#include "PCU/Data/Data.hpp"

class CurrentSensors{
    public: 

    inline static float actual_current_sensor_u_a{0.0f};
    inline static float actual_current_sensor_u_b{0.0f};
    inline static float actual_current_sensor_v_a{0.0f};
    inline static float actual_current_sensor_v_b{0.0f};
    inline static float actual_current_sensor_w_a{0.0f};
    inline static float actual_current_sensor_w_b{0.0f};
    #if PCU_H10 == 0
    
    inline static bool is_on{false};
    #endif


    private:
        inline static LinearSensor<float>* sensor_u_a{nullptr};
        inline static LinearSensor<float>* sensor_u_b{nullptr};
        inline static LinearSensor<float>* sensor_v_a{nullptr};
        inline static LinearSensor<float>* sensor_v_b{nullptr};
        inline static LinearSensor<float>* sensor_w_a{nullptr};
        inline static LinearSensor<float>* sensor_w_b{nullptr};
        
    public:

    CurrentSensors()=default;
    static void init(ST_LIB::ADCDomain::Instance& Sensor_U_A,ST_LIB::ADCDomain::Instance& Sensor_U_B,ST_LIB::ADCDomain::Instance& Sensor_V_A,
    ST_LIB::ADCDomain::Instance& Sensor_V_B,ST_LIB::ADCDomain::Instance& Sensor_W_A,ST_LIB::ADCDomain::Instance& Sensor_W_B);
    static void read();
    static void zeroing();

};