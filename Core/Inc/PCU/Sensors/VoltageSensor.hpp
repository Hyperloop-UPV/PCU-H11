#pragma once
#include "PCU/Data/Data.hpp"

class VoltageSensors{
    public: 

    struct Data_voltage
    {
        public:
        float actual_voltage_battery_a;
        float actual_voltage_battery_b;
    };

    D1_NC inline static Data_voltage data_voltage{0.0f,0.0f};

    private:
        inline static MovingAverage<50> filter_voltage_A;
        inline static MovingAverage<50> filter_voltage_B;
        inline static FilteredLinearSensor<float,50> sensor_battery_a{Pinout::Voltage_Battery_A,Sensors_data::slope_voltage_sensor,Sensors_data::offset_voltage_sensor,&data_voltage.actual_voltage_battery_a,filter_voltage_A};
        inline static FilteredLinearSensor<float,50> sensor_battery_b{Pinout::Voltage_Battery_B,Sensors_data::slope_voltage_sensor,Sensors_data::offset_voltage_sensor,&data_voltage.actual_voltage_battery_b,filter_voltage_B};
        
    public:
    
    VoltageSensors()=default;
    static void read()
    {
        sensor_battery_a.read();
        sensor_battery_b.read();
    };

};