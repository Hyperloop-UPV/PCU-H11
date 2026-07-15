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
#include "PCU/Sensors/IMU/IMU.hpp"
#include "PCU/Comms/Comms.hpp"

#define MODULATION_FREQUENCY_DEFAULT 10
#define Protecction_Voltage 410.0f 

extern ST_LIB::EthernetDomain::Instance *ethernet;
extern bool initialized_stlib;
extern const char ADJ_COMMIT_HASH[16];
extern HeapOrder *adj_commit_hash_order;

bool check_adj_commit_success();

class PCU
{
    public:
    inline static Control_Data control_data{};

    inline static bool flag_update_speed_control{false};
    inline static bool flag_update_current_control{false};
    inline static bool flag_execute_space_vector_control{false};
    inline static bool flag_sensors_update{false};
    inline static bool flag_check_transitions{false};
    inline static bool flag_speetec_update{false};
    inline static bool callback_flag{false};


    inline static States_PCU current_state_pcu{States_PCU::Connecting};
    inline static Operational_States_PCU current_operational_state_pcu{Operational_States_PCU::IDLE};
    inline static States_Shown_PCU current_sending_state{States_Shown_PCU::Connecting};

    static void start();
    static void update();

    
    private:
    static void stop_motors();
    static void start_precharge();
    static void motor_brake();
    static void initialize_protections();
    
    public:


/*-----State Machine declaration------*/

static constexpr auto connecting_state = make_state(States_PCU::Connecting,
    Transition<States_PCU>{States_PCU::Operational,[]()
    {
        return OrderPackets::TCP_CONNECTION->is_connected() && check_adj_commit_success();
    }}
);

static constexpr auto operational_state = make_state(States_PCU::Operational,
    Transition<States_PCU>{States_PCU::Fault,[]()
    {
        return (!OrderPackets::TCP_CONNECTION->is_connected() || !ethernet->is_connected());
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

//Por hacer regenerativo...

static inline constinit auto Operational_State_Machine = []() consteval
{
    auto sm= make_state_machine(Operational_States_PCU::IDLE,
        nested_idle_state,
        nested_accelerating_state
    );
    using namespace std::chrono_literals;

    sm.add_enter_action([]()
    {
        callback_flag = false;
        // IMU::restart();
        stop_motors();
    },nested_idle_state);

    sm.add_enter_action([]()
    {
        callback_flag = true;
    },nested_accelerating_state);

    sm.add_exit_action([]()
    {
        callback_flag = false;
    },nested_accelerating_state);

    sm.add_cyclic_action([]()
    {
        if(SpeedControl::running)
        {
            SpeedControl::control_action();
        }
    }, us(Speed_Control_Data::microsecond_period) , nested_accelerating_state);


    // sm.add_cyclic_action([]() Implemented on the interuption
    // {   
    //     if(control_data.space_vector_active == SpaceVectorState::ACTIVE)
    //     {
    //         SpaceVector::calculate_duties();
    //     }
    //     if(CurrentControl::is_running())
    //     {
    //         CurrentControl::control_action();
    //     }
    // }, us(Current_Control_Data::microsecond_period) , nested_accelerating_state);


    
    sm.add_enter_action([]()
    {
        #if PCU_H10 == 0
        Actuators::set_led_accelerating(true);
        #else
        Actuators::enable_reset_bypass();
        #endif
        
        Actuators::enable_buffer();
    }, nested_accelerating_state);

    sm.add_exit_action([]()
    {
        #if PCU_H10 == 0
        Actuators::set_led_braking(true);
        Actuators::set_led_accelerating(false);
        #else
        Actuators::disable_reset_bypass();
        #endif
        PWMActuators::stop();
        Actuators::disable_buffer();
    }, nested_accelerating_state);
    return sm;
}();

static inline constinit auto PCU_State_Machine = []() consteval
{
    auto nested = StateMachineHelper::add_nesting(operational_state, Operational_State_Machine);
    auto sm = make_state_machine(States_PCU::Connecting,
        StateMachineHelper::add_nested_machines(nested),
        connecting_state,
        operational_state,
        fault_state

    );
    using namespace std::chrono_literals;
    sm.add_cyclic_action([]()
    {
        static bool toggle = true;
        Actuators::set_led_connecting(toggle);
        toggle = !toggle;
    }, ms(500), connecting_state);

    sm.add_enter_action([]()
    {
        Actuators::set_led_connecting(true);
        Actuators::set_led_operational(true);
        Actuators::set_led_fault(false);
    }, operational_state);

    sm.add_exit_action([]()
    {
        stop_motors();
        Actuators::set_led_connecting(false);
        callback_flag = false;
    }, operational_state);

    sm.add_enter_action([]()
    {
        callback_flag = false;
        if(initialized_stlib) {
            stop_motors();
            // ProtectionManager::propagate_fault();
            Actuators::set_led_operational(false);
            Actuators::set_led_connecting(false);
            Actuators::set_led_fault(true);
        } else {
            WARNING("Fault in board::init()");
        }
    }, fault_state);


    return sm;
}();


};

namespace PCU_Protections {

    inline void on_fault() {
        PCU::PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
    }

    inline constexpr auto voltage_A = Protections::protection<
        "PCU Battery A voltage", VoltageSensors::actual_voltage_battery_a>(
        Protections::Rules::above<float>(Protecction_Voltage)
    );

    inline constexpr auto voltage_B = Protections::protection<
        "PCU Battery B voltage", VoltageSensors::actual_voltage_battery_b>(
        Protections::Rules::above<float>(Protecction_Voltage)
    );

    inline constexpr auto current_u_a = Protections::protection<
        "PCU Current Sensor U A", CurrentSensors::actual_current_sensor_u_a>(
        Protections::Rules::above<float>(CURRENT_PROTECTION)
    );

    inline constexpr auto current_v_a = Protections::protection<
        "PCU Current Sensor V A", CurrentSensors::actual_current_sensor_v_a>(
        Protections::Rules::above<float>(CURRENT_PROTECTION)
    );

    inline constexpr auto current_w_a = Protections::protection<
        "PCU Current Sensor W A", CurrentSensors::actual_current_sensor_w_a>(
        Protections::Rules::above<float>(CURRENT_PROTECTION)
    );

    inline constexpr auto current_u_b = Protections::protection<
        "PCU Current Sensor U B", CurrentSensors::actual_current_sensor_u_b>(
        Protections::Rules::above<float>(CURRENT_PROTECTION)
    );

    inline constexpr auto current_v_b = Protections::protection<
        "PCU Current Sensor V B", CurrentSensors::actual_current_sensor_v_b>(
        Protections::Rules::above<float>(CURRENT_PROTECTION)
    );

    inline constexpr auto current_w_b = Protections::protection<
        "PCU Current Sensor W B", CurrentSensors::actual_current_sensor_w_b>(
        Protections::Rules::above<float>(CURRENT_PROTECTION)
    );

    inline constexpr auto position_encoder = Protections::protection<
        "PCU Position", PCU::control_data.IMU_position_m>(
        Protections::Rules::above<float>(33.0)
    );

    inline constexpr auto space_vector_time = Protections::protection<
        "PCU Space Vector Time", PCU::control_data.time>(
        Protections::Rules::above<float>(4.5f)
    );
}

using PCUFaultPolicy = ST_LIB::FaultPolicy<PCU::PCU_State_Machine, PCU_Protections::on_fault>;