#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"
#include "PCU/Sensors/CurrentSensor.hpp"
#include "PCU/Sensors/VoltageSensor.hpp"
#include "PCU/Sensors/Speetec.hpp"
#include "PCU/Actuators/Actuators.hpp"
#include "PCU/Actuators/PWMActuators.hpp"
#include "PCU/Control/SpaceVector.hpp"
#include "PCU/Control/CurrentControl.hpp"
#include "PCU/Control/SpeedControl.hpp"
#include "PCU/Comms/Comms.hpp"

#define MODULATION_FREQUENCY_DEFAULT 10
#define Protecction_Voltage 325.0f 
class PCU
{
    public:
    D1_NC inline static Control_Data control_data{};
    inline static bool space_vector_on{false};
    inline static bool speed_control_on{false};
    inline static bool current_control_on{false};




    /*-----State Machine declaration------*/

    static constexpr auto connecting_state = make_state(States_PCU::Connecting,
        Transition<States_PCU>{States_PCU::Operational,[]()
        {
            return DataPackets::control_station_tcp->is_connected();
        }},
        Transition<States_PCU>{States_PCU::Fault,[]()
        {
            return (VoltageSensors::actual_voltage_battery_a > Protecction_Voltage) || (VoltageSensors::actual_voltage_battery_b > Protecction_Voltage);
        }}
    );

    static constexpr auto operational_state = make_state(States_PCU::Operational,
        Transition<States_PCU>{States_PCU::Fault,[]()
        {
            return !DataPackets::control_station_tcp->is_connected() || (VoltageSensors::actual_voltage_battery_a > Protecction_Voltage) || (VoltageSensors::actual_voltage_battery_b > Protecction_Voltage);
        }}
    );

    static constexpr auto fault_state = make_state(States_PCU::Fault);

    static constexpr auto nested_idle_state = make_state(Operational_States_PCU::Idle,
        Transition<Operational_States_PCU>{Operational_States_PCU::Sending_PWM,[]()
        {
            return control_data.pwm_active != PWM_ACTIVE::NONE && control_data.buffer_state == BUFFER_STATE::ENABLED;
        }},
        Transition<Operational_States_PCU>{Operational_States_PCU::Accelerating,[]()
        {
            return space_vector_on;
        }}
    );

    static constexpr auto nested_sending_pwm_state = make_state(Operational_States_PCU::Sending_PWM,
        Transition<Operational_States_PCU>{Operational_States_PCU::Idle,[]()
        {
            return control_data.pwm_active == PWM_ACTIVE::NONE || control_data.buffer_state == BUFFER_STATE::DISABLED;
        }}

    );

    
    
};