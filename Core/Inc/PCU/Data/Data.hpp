#pragma once
#include "ST-LIB.hpp"

/*-------Flags-------*/
#define PCU_H10 0           // 0 = PCU-H11, 1 = PCU-H10
#define MODE_CALCULATE_SIN 1 //0 = Normal sin, 1 = Look Up table with direct interpolation
#define PPU_USING 1     // 0 PPU connector A, 1 PPU connector B , 2 Both PPU
#define ARMONIC_INJECTION 1 // 0 = NO armonic injection, 1 = Armonic injection active
#define MOTOR_CENTER_ALIGNED 0 // 0 = Normal dualPWM, 1 = CenterAligned DualPWM
#define SATURATOR_PI 1 // 0 = No saturator PI, 1 = Saturator PI


#if PCU_H10 == 1
namespace Pinout{
    static constexpr Pin& U_PWM = PE9;
    static constexpr Pin& U_PWM_NEGATED = PE8; 
    static constexpr Pin& V_PWM = PE11;
    static constexpr Pin& V_PWM_NEGATED = PE10; 
    static constexpr Pin& W_PWM = PE13;
    static constexpr Pin& W_PWM_NEGATED = PE12;

    constexpr DigitalOutputDomain::DigitalOutput Buff_enable{ST_LIB::PF4}; 
    constexpr DigitalOutputDomain::DigitalOutput Reset_bypass{ST_LIB::PB7};

    static constexpr Pin& Voltage_Battery_A = PF3;
    static constexpr Pin& Voltage_Battery_B = PF5;

    constexpr DigitalOutputDomain::DigitalOutput led_connecting{ST_LIB::PG6};
    constexpr DigitalOutputDomain::DigitalOutput led_fault = {ST_LIB::PG7};
    constexpr DigitalOutputDomain::DigitalOutput led_operational = {ST_LIB::PG8};
    //current sensors
    static constexpr Pin& Current_sensor_U_A = PA0;
    static constexpr Pin& Current_sensor_U_B = PA6;
    static constexpr Pin& Current_sensor_V_A = PA4;
    static constexpr Pin& Current_sensor_V_B = PB0;
    static constexpr Pin& Current_sensor_W_A = PA5;
    static constexpr Pin& Current_sensor_W_B = PB1;
    //Speetec
    static constexpr Pin& Speetec_A = PF1;
    static constexpr Pin& Speetec_B = PF0;
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
    // static constexpr Pin& Speetec_C = PE6;
    // static constexpr Pin& Speetec_C_outPhase = PE5;

    /*------Voltage Battery------*/
    static constexpr Pin& Voltage_Battery_A = PF3;
    static constexpr Pin& Voltage_Battery_B = PF5;

    /*------Temperature------*/
    static constexpr Pin& PPU_temp_A = PF7;
    static constexpr Pin& PPU_temp_B = PF8;

    /*------B2B------*/
    //Imagino que serán DI:
    // constexpr DigitalInputDomain::DigitalInput Fault_A = {ST_LIB::PE14};
    // constexpr DigitalInputDomain::DigitalInput Fault_B = {ST_LIB::PB6};
    // constexpr DigitalInputDomain::DigitalInput Ready_A = {ST_LIB::PB5};
    // constexpr DigitalInputDomain::DigitalInput Ready_B = {ST_LIB::PE14};
    //Outputs:
    constexpr DigitalOutputDomain::DigitalOutput Buff_enable = {ST_LIB::PF4}; //Negado en h10, no se si en h11 si
    constexpr DigitalOutputDomain::DigitalOutput Reset_bypass = {ST_LIB::PG5};


    /*------Supplies------*/
    constexpr DigitalOutputDomain::DigitalOutput Speetec_supply = {ST_LIB::PD10};
    constexpr DigitalOutputDomain::DigitalOutput Hall_SupplyA= {ST_LIB::PD12};
    constexpr DigitalOutputDomain::DigitalOutput Hall_SupplyB= {ST_LIB::PD11};





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

    static constexpr uint32_t read_sensors_us = 200; //This speed allows maximum precision with speetec.
    static constexpr double encoder_sample_time_s =  static_cast<double>(read_sensors_us)/1e6; // this has to be the same frequency that the read is done
    static constexpr double encoder_counter_distance_m = 0.004; // mm
    static constexpr size_t  encoder_samples = 250;
};

enum ControlStates{
    accelerate,
    regenerate
};

enum class PWM_ACTIVE: uint8_t
{
    NONE = 0,
    U = 1,
    V = 2,
    W = 3
}; 

enum class BUFFER_STATE: uint8_t
{
    DISABLED = 0,
    ENABLED = 1
};
using Direction = EncoderSensor<Sensors_data::encoder_samples>::Direction;
struct Control_Data
{
    PWM_ACTIVE pwm_active{};
    uint32_t actual_frequency{};
    float modulation_frequency{};
    float actual_duty{};
    BUFFER_STATE buffer_state{};

    double current_error{};
    double current_Peak{};
    double target_voltage{};
    float time{};
    float imod{};
    //speed
    float target_speed{};
    double speed_error{};
    float actual_current_ref{};
    //control
    ControlStates currentState{ControlStates::accelerate};
    ControlStates speedState{ControlStates::accelerate};
    Direction established_direction{Direction::FORWARD};
};