#pragma once
#include "ST-LIB.hpp"

/*-------Flags-------*/
#define PCU_H10 0


#if PCU_H10 == 1
namespace Pinout{
    static constexpr Pin& U_PWM = PE9;
    static constexpr Pin& U_PWM_NEGATED = PE8; 
    static constexpr Pin& V_PWM = PE11;
    static constexpr Pin& V_PWM_NEGATED = PE10; 
    static constexpr Pin& W_PWM = PE13;
    static constexpr Pin& W_PWM_NEGATED = PE12;

    static constexpr Pin& ENABLE_BUFFER = PF4; 
    static constexpr Pin& Reset = PB7;
    static constexpr Pin& Batt_Voltage_A = PF3;
    static constexpr Pin& Batt_Voltage_B = PF5;
    static constexpr Pin& LED_COMMUTATION = PG6;
    static constexpr Pin& LED_FAULT = PG7;
    static constexpr Pin& LED_OPERATIONAL = PG8;
    //current sensors
    static constexpr Pin& CURRENT_SENSOR_U_A = PA0;
    static constexpr Pin& CURRENT_SENSOR_U_B = PA6;
    static constexpr Pin& CURRENT_SENSOR_V_A = PA4;
    static constexpr Pin& CURRENT_SENSOR_V_B = PB0;
    static constexpr Pin& CURRENT_SENSOR_W_A = PA5;
    static constexpr Pin& CURRENT_SENSOR_W_B = PB1;
    //Speetec
    static constexpr Pin& ENCODER_A = PF1;
    static constexpr Pin& ENCODER_B = PF0;
};
#else 
using ST_LIB::DigitalInputDomain;
using ST_LIB::DigitalOutputDomain;

namespace Pinout
{
    /*------Leds------*/
    constexpr DigitalOutputDomain::DigitalOutput led_operational = {ST_LIB::PG13};
    constexpr DigitalOutputDomain::DigitalOutput led_fault = {ST_LIB::PG12};    
    constexpr DigitalOutputDomain::DigitalOutput led_connecting = {ST_LIB::PG11};
    constexpr DigitalOutputDomain::DigitalOutput led_accelerating = {ST_LIB::PG9};
    constexpr DigitalOutputDomain::DigitalOutput led_braking = {ST_LIB::PG6};

    /*------Current Sensors------*/
    static constexpr Pin& Current_sensor_U_A = PA0;
    static constexpr Pin& Current_sensor_U_B = PA6;
    static constexpr Pin& Current_sensor_V_A = PA4;
    static constexpr Pin& Current_sensor_V_B = PB0;
    static constexpr Pin& Current_sensor_W_A = PA5;
    static constexpr Pin& Current_sensor_W_B = PB1;
    
    /*------PWM------*/
    static constexpr Pin& U_PWM = PE9;
    static constexpr Pin& U_PWM_NEGATED = PE8;
    static constexpr Pin& V_PWM = PE11;
    static constexpr Pin& V_PWM_NEGATED = PE10;
    static constexpr Pin& W_PWM = PE13;
    static constexpr Pin& W_PWM_NEGATED = PE12;

    /*------Encoder------*/
    static constexpr Pin& Speetec_A = PF1;
    static constexpr Pin& Speetec_A_outPhase = PF0;
    static constexpr Pin& Speetec_B = PF9;
    static constexpr Pin& Speetec_B_outPhase = PF2;
    static constexpr Pin& Speetec_C = PE6;
    static constexpr Pin& Speetec_C_outPhase = PE5;

    /*------Voltage Battery------*/
    static constexpr Pin& Voltage_Battery_A = PF3;
    static constexpr Pin& Voltage_Battery_B = PF5;

    /*------Temperature------*/
    static constexpr Pin& PPU_temp_A = PF7;
    static constexpr Pin& PPU_temp_B = PF8;

    /*------B2B------*/
    //Imagino que serán DI:
    constexpr DigitalInputDomain::DigitalInput Fault_A = {ST_LIB::PE14};
    constexpr DigitalInputDomain::DigitalInput Fault_B = {ST_LIB::PB6};
    constexpr DigitalInputDomain::DigitalInput Ready_A = {ST_LIB::PB5};
    constexpr DigitalInputDomain::DigitalInput Ready_B = {ST_LIB::PE14};

    constexpr DigitalOutputDomain::DigitalOutput Buff_enable = {ST_LIB::PF4};
    constexpr DigitalOutputDomain::DigitalOutput Reset_bypass = {ST_LIB::PG5};


    //IDK :)
    static constexpr Pin& Hal_supply_A = PD12;
    static constexpr Pin& Hal_supply_B = PD11;
    static constexpr Pin& Speetec_supply = PD10;




}
#endif

namespace Current_Control_Data
{
    static constexpr double kp_accelerate = 0.1;
    static constexpr double ki_accelerate = 12.0;
    static constexpr double kp_regenerate = 0.2;
    static constexpr double ki_regenerate = 6.0;
    static constexpr double period = 0.0002;
    //Esto por ver:
    static constexpr uint32_t microsecond_period = static_cast<uint32_t>(period * 1000000);
}
namespace Speed_Control_Data
{
    static constexpr double kp_accelerate = 4.0;
    static constexpr double ki_accelerate = 1.0;
    static constexpr double kp_regenerate = 4.0;
    static constexpr double ki_regenerate = 1.0;
    static constexpr double period = 0.1;
    static constexpr uint32_t microsecond_period = static_cast<uint32_t>(period * 1000000);
}

namespace Sensors_data
{
    constexpr static float slope_current_sensor{96.206615f};
    constexpr static float offset_current_sensor{-159.5747f};

    constexpr static float slope_current_sensor_inverted{-97.134615384615700f};
    constexpr static float offset_current_sensor_inverted{160.660653846154000f};
    
    static constexpr float slope_voltage_sensor = 133.31627;
    static constexpr float offset_voltage_sensor = -9.24655;

    static constexpr uint32_t read_sensors_us = 200; 
    static constexpr double encoder_sample_time_s =  static_cast<double>(read_sensors_us)/1e6; // this has to be the same frequency that the read is done
    static constexpr double encoder_counter_distance_m = 0.0001;
    static constexpr size_t  encoder_samples = 250;
};