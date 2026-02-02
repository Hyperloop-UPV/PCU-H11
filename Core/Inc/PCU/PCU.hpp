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
#include "PCU/Control/Runs.hpp"

#define MODULATION_FREQUENCY_DEFAULT 10
#define Protecction_Voltage 325.0f 
class PCU
{
    public:
    D1_NC inline static Control_Data control_data{};
    inline static DataPackets::space_vector_active space_vector_on{DataPackets::space_vector_active::DISABLE};
    inline static DataPackets::speed_control_active speed_control_on{DataPackets::speed_control_active::DISABLE};
    inline static DataPackets::current_control_active current_control_on{DataPackets::current_control_active::DISABLE};

    inline static bool flag_update_speed_control{false};
    inline static bool flag_update_current_control{false};

    static void start();
    static void update();





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
    Transition<Operational_States_PCU>{Operational_States_PCU::Regenerative,[]()
    {
        return control_data.pwm_active != PWM_ACTIVE::NONE && control_data.buffer_state == BUFFER_STATE::ENABLED;
    }},
    Transition<Operational_States_PCU>{Operational_States_PCU::Accelerating,[]()
    {
        return space_vector_on == DataPackets::space_vector_active::ACTIVE;
    }}
);

static constexpr auto nested_braking_state = make_state(Operational_States_PCU::Braking); //Me imagino que llegará orden de la vcu, por ahora ahi se queda.

static constexpr auto nested_sending_pwm_state = make_state(Operational_States_PCU::Regenerative,
    Transition<Operational_States_PCU>{Operational_States_PCU::IDLE,[]()
    {
        return control_data.pwm_active == PWM_ACTIVE::NONE || control_data.buffer_state == BUFFER_STATE::DISABLED;
    }}

);

static constexpr auto nested_accelerating_state = make_state(Operational_States_PCU::Accelerating,
    Transition<Operational_States_PCU>{Operational_States_PCU::IDLE,[]()
    {
        return space_vector_on == DataPackets::space_vector_active::DISABLE;
    }}
);

static inline constinit auto Operational_State_Machine = []() consteval
{
    auto sm= make_state_machine(Operational_States_PCU::IDLE,
        nested_idle_state,
        nested_braking_state,
        nested_accelerating_state,
        nested_sending_pwm_state
    );
    using namespace std::chrono_literals;
    sm.add_cyclic_action([]()
    {
        flag_update_speed_control = true;
    }, us(Speed_Control_Data::microsecond_period) , nested_accelerating_state);

    sm.add_cyclic_action([]()
    {
        flag_update_current_control = true;
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

    sm.add_enter_action([]()
    {
        PWMActuators::stop();
        Actuators::disable_buffer();
        Actuators::disable_reset_bypass();
        
        Actuators::set_led_operational(false);
        Actuators::set_led_connecting(false);
        Actuators::set_led_fault(true);
    }, fault_state);

    return sm;
}();


inline static DataPackets data_packets{control_data.actual_frequency,control_data.modulation_frequency,PWMActuators::duty_cycle_u,PWMActuators::duty_cycle_v,PWMActuators::duty_cycle_w,
    VoltageSensors::actual_voltage_battery_a,VoltageSensors::actual_voltage_battery_b,
    CurrentSensors::actual_current_sensor_u_a,CurrentSensors::actual_current_sensor_v_a,CurrentSensors::actual_current_sensor_w_a,CurrentSensors::actual_current_sensor_u_b,CurrentSensors::actual_current_sensor_v_b,CurrentSensors::actual_current_sensor_w_b,control_data.current_Peak,control_data.current_error,
    control_data.target_voltage,SpaceVector::time,control_data.imod,
    PCU_State_Machine.current_state,Operational_State_Machine.current_state,
    control_data.state_run,space_vector_on,current_control_on,speed_control_on,
    Speetec::position_encoder,control_data.established_direction,Speetec::speed_encoder,control_data.speed_km_h_encoder,Speetec::acceleration_encoder,control_data.target_speed,control_data.speed_error,
    control_data.actual_current_ref,control_data.direction_state,control_data.speedState,
    Sensors::gd_fault_a,Sensors::gd_fault_b,Sensors::gd_ready_a,Sensors::gd_ready_b};
    
    
};