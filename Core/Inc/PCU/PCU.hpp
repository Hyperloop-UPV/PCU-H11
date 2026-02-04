#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"
#include "PCU/Sensors/CurrentSensor.hpp"
#include "PCU/Sensors/VoltageSensor.hpp"
#include "PCU/Sensors/Speetec.hpp"
#include "PCU/Sensors/Sensors.hpp"
#include "PCU/Actuators/Actuators.hpp"
#include "PCU/Actuators/PWMActuators.hpp"
#include "PCU/Control/SpaceVector.hpp"
#include "PCU/Control/CurrentControl.hpp"
#include "PCU/Control/SpeedControl.hpp"
#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"
#include "PCU/Sensors/Sensors.hpp"
#include "PCU/Comms/Comms.hpp"

#define MODULATION_FREQUENCY_DEFAULT 10
#define Protecction_Voltage 325.0f 
class PCU
{
    public:
    D1_NC inline static Control_Data control_data{};

    inline static bool flag_update_speed_control{false};
    inline static bool flag_update_current_control{false};
    inline static bool flag_execute_space_vector_control{false};

    inline static States_PCU current_state_pcu{States_PCU::Connecting};
    inline static Operational_States_PCU current_operational_state_pcu{Operational_States_PCU::IDLE};

    static void start();
    static void update();

    
    private:
    static void stop_motors();
    static void start_precharge();
    static void motor_brake();
    public:


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

static constexpr auto nested_idle_state = make_state(Operational_States_PCU::IDLE,
    Transition<Operational_States_PCU>{Operational_States_PCU::Accelerating,[]()
    {
        return control_data.space_vector_active == SpaceVectorState::ACTIVE;
    }}
);


static constexpr auto nested_accelerating_state = make_state(Operational_States_PCU::Accelerating,
    Transition<Operational_States_PCU>{Operational_States_PCU::IDLE,[]()
    {
        return control_data.space_vector_active == SpaceVectorState::DISABLE;
    }}
);

//Por hacer regenerativo

static inline constinit auto Operational_State_Machine = []() consteval
{
    auto sm= make_state_machine(Operational_States_PCU::IDLE,
        nested_idle_state,
        nested_accelerating_state
    );
    using namespace std::chrono_literals;

    sm.add_enter_action([]()
    {
        stop_motors();
    },nested_idle_state);

    sm.add_cyclic_action([]()
    {
        if(SpeedControl::running)
        {
            flag_update_speed_control = true;
        }
    }, us(Speed_Control_Data::microsecond_period) , nested_accelerating_state);

    sm.add_cyclic_action([]()
    {   
        flag_execute_space_vector_control = true;
        if(CurrentControl::is_running())flag_update_current_control = true;
    }, us(Current_Control_Data::microsecond_period) , nested_accelerating_state);

    
    sm.add_enter_action([]()
    {
        #if PCU_H10 == 0
        Actuators::set_led_accelerating(true);
        Actuators::enable_hall_supply();
        Actuators::enable_speedtec_supply(); //No se si poner esto en operational o aqui.
        #endif
        Actuators::enable_buffer();
    }, nested_accelerating_state);

    sm.add_exit_action([]()
    {
        #if PCU_H10 == 0
        Actuators::set_led_braking(true);
        Actuators::set_led_accelerating(false);
        // Actuators::disable_hall_supply();
        // Actuators::disable_speedtec_supply();
        #endif
        PWMActuators::stop();
        Actuators::disable_buffer();
    }, nested_accelerating_state);
    return sm;
}();

static inline constinit auto PCU_State_Machine = []() consteval
{
    auto sm = make_state_machine(States_PCU::Connecting,
        connecting_state,
        operational_state,
        fault_state
    );

    sm.add_exit_action([]()
    {
        Actuators::set_led_connecting(false);
    }, connecting_state);

    sm.add_enter_action([]()
    {
        CurrentSensors::zeroing();
        Actuators::set_led_connecting(true);
    }, connecting_state);

    sm.add_enter_action([]()
    {
        Actuators::set_led_connecting(false);
        Actuators::set_led_operational(true);
        Actuators::set_led_fault(true);
    }, operational_state);

    sm.add_exit_action([]()
    {
        stop_motors();
        Actuators::set_led_operational(false);
    }, operational_state);

    sm.add_enter_action([]()
    {
        stop_motors();

        Actuators::set_led_operational(false);
        Actuators::set_led_connecting(false);
        Actuators::set_led_fault(true);
    }, fault_state);

    sm.add_state_machine(Operational_State_Machine, operational_state);

    return sm;
}();


};