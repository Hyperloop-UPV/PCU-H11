#pragma once
#include "ST-LIB.hpp"

/*-------Flags-------*/
#define PCU_H10 1        // 0 = PCU-H11, 1 = PCU-H10
#define MODE_CALCULATE_SIN 1 //0 = Normal sin, 1 = Look Up table with direct interpolation
#define PPU_USING 1     // 0 PPU connector A, 1 PPU connector B , 2 Both PPU
#define ARMONIC_INJECTION 1 // 0 = NO armonic injection, 1 = Armonic injection active
#define MOTOR_CENTER_ALIGNED 0 // 0 = Normal dualPWM, 1 = CenterAligned DualPWM
#define SATURATOR_PI 1 // 0 = No saturator PI, 1 = Saturator PI
#define PWM_TEST_MODE 1 // 0 = Normal mode, 1 = PWM test mode

using ST_LIB::DigitalInputDomain;
using ST_LIB::DigitalOutputDomain;
using ST_LIB::TimerDomain;

#if PCU_H10 == 1
namespace Pinout{
    inline constexpr ST_LIB::TimerPin U_PWM_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE9,
    .channel = ST_LIB::TimerChannel::CHANNEL_1,
    };

    inline constexpr ST_LIB::TimerPin U_PWM_negated_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE8,
    .channel = ST_LIB::TimerChannel::CHANNEL_1_NEGATED,
    };


    inline constexpr ST_LIB::TimerPin V_PWM_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE11,
    .channel = ST_LIB::TimerChannel::CHANNEL_2,
    };

    inline constexpr ST_LIB::TimerPin V_PWM_negated_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE10,
    .channel = ST_LIB::TimerChannel::CHANNEL_2_NEGATED,
    };

    
    inline constexpr ST_LIB::TimerPin W_PWM_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE13,
    .channel = ST_LIB::TimerChannel::CHANNEL_3,
    };

    inline constexpr ST_LIB::TimerPin W_PWM_negated_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE12,
    .channel = ST_LIB::TimerChannel::CHANNEL_3_NEGATED,
    };

    inline constexpr ST_LIB::TimerDomain::Timer tim_decl{{
    .request = ST_LIB::TimerRequest::Advanced_1,
    }, U_PWM_pin, U_PWM_negated_pin, V_PWM_pin, V_PWM_negated_pin, W_PWM_pin, W_PWM_negated_pin};

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

    constexpr DigitalInputDomain::DigitalInput FAULT_GD_INVERTER_A = {ST_LIB::PB6};
    constexpr DigitalInputDomain::DigitalInput FAULT_GD_INVERTER_B = {ST_LIB::PE15};
    constexpr DigitalInputDomain::DigitalInput READY_GD_INVERTER_A = {ST_LIB::PB5};
    constexpr DigitalInputDomain::DigitalInput READY_GD_INVERTER_B = {ST_LIB::PE14};
};
#else 


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
    inline constexpr ST_LIB::TimerPin U_PWM_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE9,
    .channel = ST_LIB::TimerChannel::CHANNEL_1,
    };

    inline constexpr ST_LIB::TimerPin U_PWM_negated_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE8,
    .channel = ST_LIB::TimerChannel::CHANNEL_1_NEGATED,
    };


    inline constexpr ST_LIB::TimerPin V_PWM_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE11,
    .channel = ST_LIB::TimerChannel::CHANNEL_2,
    };

    inline constexpr ST_LIB::TimerPin V_PWM_negated_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE10,
    .channel = ST_LIB::TimerChannel::CHANNEL_2_NEGATED,
    };

    
    inline constexpr ST_LIB::TimerPin W_PWM_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE13,
    .channel = ST_LIB::TimerChannel::CHANNEL_3,
    };

    inline constexpr ST_LIB::TimerPin W_PWM_negated_pin {
    .af = ST_LIB::TimerAF::PWM,
    .pin = ST_LIB::PE12,
    .channel = ST_LIB::TimerChannel::CHANNEL_3_NEGATED,
    };

    inline constexpr ST_LIB::TimerDomain::Timer tim_decl{{
    .request = ST_LIB::TimerRequest::Advanced_1,
    }, U_PWM_pin, U_PWM_negated_pin};



    /*------Encoder------*/
    static constexpr Pin& Speetec_A = PF1;
    static constexpr Pin& Speetec_A_outPhase = PF0;
    static constexpr Pin& Speetec_B = PF9;
    static constexpr Pin& Speetec_B_outPhase = PF2;

    // static constexpr ST_LIB::TimerDomain::Timer encoder_timer{
    // ST_LIB::TimerRequest::GeneralPurpose32bit_23, 
    // {'e', 'n', 'c', 'o', 'd', 'e', 'r', '\0'},    
    // ST_LIB::TimerPin{ST_LIB::TimerAF::Encoder, ST_LIB::PF1, ST_LIB::TimerChannel::CHANNEL_2},
    // ST_LIB::TimerPin{ST_LIB::TimerAF::Encoder, ST_LIB::PF9, ST_LIB::TimerChannel::CHANNEL_4}
    // };
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
    constexpr DigitalInputDomain::DigitalInput FAULT_GD_INVERTER_A = {ST_LIB::PE14};
    constexpr DigitalInputDomain::DigitalInput FAULT_GD_INVERTER_B = {ST_LIB::PB6};
    constexpr DigitalInputDomain::DigitalInput READY_GD_INVERTER_A = {ST_LIB::PB5};
    constexpr DigitalInputDomain::DigitalInput READY_GD_INVERTER_B = {ST_LIB::PE14};
    // Outputs:
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

#include "Communications/Packets/DataPackets.hpp"

using States_PCU = DataPackets::general_state_machine;
using Operational_States_PCU = DataPackets::operational_state_machine;
using EncoderDirection = DataPackets::encoder_direction;
using PWM_ACTIVE = DataPackets::space_vector_active;
using SpeedControlState = DataPackets::speed_control_active;
using CurrentControlState = DataPackets::current_control_active;
using SpaceVectorState = DataPackets::space_vector_active;

enum class BUFFER_STATE: uint8_t
{
    DISABLED = 0,
    ENABLED = 1
};
struct Control_Data
{
    //SpaceVector:
    PWM_ACTIVE pwm_active{};
    uint32_t actual_frequency{};
    float modulation_frequency{0.0f};
    float duty_cycle_u{0.0f};
    float duty_cycle_v{0.0f};
    float duty_cycle_w{0.0f};
    BUFFER_STATE buffer_state{BUFFER_STATE::DISABLED};

    double current_error{0.0f};
    double current_Peak{0.0f};
    double target_voltage{0.0f};
    float time{0.0f};
    float imod{0.0f};
    //speed:
    float target_speed{0.0f};
    double speed_error{0.0f};
    float actual_current_ref{0.0f};
    double speed_km_h_encoder{0.0f};
    
    double position_encoder{0.0f}; 

    EncoderDirection established_direction{EncoderDirection::Forward};
    SpeedControlState speed_control_active{false};
    CurrentControlState current_control_active{false};
    SpaceVectorState space_vector_active{false};
};