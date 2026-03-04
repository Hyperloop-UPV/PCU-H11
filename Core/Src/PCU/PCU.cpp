#include "PCU/PCU.hpp"

void PCU::start()
{
    Comms::start();
    CurrentControl::init();
    SpeedControl::init();
    PCU_State_Machine.start();

    Scheduler::register_task(Sensors_data::read_sensors_us, [](){
        flag_speetec_update = true;
    });

    Scheduler::register_task(400, [](){
        flag_sensors_update = true;
    });

    Scheduler::register_task(100, [](){
        flag_check_transitions = true;
    });




    control_data.space_vector_active = SpaceVectorState::DISABLE;
    control_data.speed_control_active = SpeedControlState::DISABLE;
    control_data.current_control_active = CurrentControlState::DISABLE;
    initialize_protections();

    Scheduler::register_task(600, [](){
        ProtectionManager::check_protections();
    });

    #if PCU_H10 == 0
    Actuators::enable_hall_supply();
    Actuators::enable_speedtec_supply();
    #endif

}

void PCU::initialize_protections()
{   
    ProtectionManager::link_state_machine(PCU_State_Machine,
                                        static_cast<uint8_t>(States_PCU::Fault));
    
    ProtectionManager::add_standard_protections();

    auto& voltage_protection_A_PCU = ProtectionManager::_add_protection(
    &VoltageSensors::actual_voltage_battery_a, Boundary<float, ABOVE>{Protecction_Voltage});

    voltage_protection_A_PCU.set_name((char*)"PCU Battery A voltage");

    auto& voltage_protection_B_PCU = ProtectionManager::_add_protection(
    &VoltageSensors::actual_voltage_battery_b, Boundary<float, ABOVE>{Protecction_Voltage});

    voltage_protection_B_PCU.set_name((char*)"PCU Battery B voltage");

    auto& current_protection_u_a = ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_u_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    current_protection_u_a.set_name((char*)"PCU Current Sensor U A");

    auto& current_protection_v_a = ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_v_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    current_protection_v_a.set_name((char*)"PCU Current Sensor V A");
    
    auto& current_protection_w_a = ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_w_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    current_protection_w_a.set_name((char*)"PCU Current Sensor W A");

    auto& current_protection_u_b = ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_u_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    current_protection_u_b.set_name((char*)"PCU Current Sensor U B");
    
    auto& current_protection_v_b = ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_v_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    current_protection_v_b.set_name((char*)"PCU Current Sensor V B");
    
    auto& current_protection_w_b = ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_w_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    current_protection_w_b.set_name((char*)"PCU Current Sensor W B");

    
    ProtectionManager::initialize();

}

void PCU::stop_motors()
{
    flag_update_current_control = false;
    flag_update_speed_control = false;
    CurrentControl::stop();
    SpeedControl::stop();
    PWMActuators::stop();
    Actuators::disable_buffer();
    SpeedControl::set_reference_speed(0);
    PWMActuators::set_three_frequencies(0);
    SpaceVector::set_VMAX(0);
    Operational_State_Machine.force_change_state(nested_idle_state);
}

void PCU::update()
{
    if(flag_check_transitions)
    {
        flag_check_transitions=false;
        PCU_State_Machine.check_transitions();
        current_state_pcu = PCU_State_Machine.get_current_state();
        current_operational_state_pcu = Operational_State_Machine.get_current_state();
    }

    if(OrderPackets::Stop_Motor_flag == true)
    {
        OrderPackets::Stop_Motor_flag=false;
        stop_motors();
        control_data.space_vector_active = SpaceVectorState::DISABLE;
    }

    if(flag_speetec_update)
    {
        flag_speetec_update=false;
        Speetec::read();
    }

    if(flag_sensors_update)
    {
        flag_sensors_update=false;
        Sensors::read();
        VoltageSensors::read();
        CurrentSensors::read();

    }
    
    if(current_state_pcu == States_PCU::Fault)
    {
        return;
    }
    
    if(OrderPackets::Start_SVPWM_flag == true)
    {
        OrderPackets::Start_SVPWM_flag=false;
        control_data.space_vector_active=SpaceVectorState::ACTIVE;
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_frequency_Modulation(Comms::frequency_space_vector_received);
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        SpaceVector::set_target_voltage(Comms::ref_voltage_space_vector_received);
        SpaceVector::reset_time();

        CurrentControl::stop();
        SpeedControl::stop();

    }

    if(OrderPackets::Send_Reference_Current_flag == true)
    {
        OrderPackets::Send_Reference_Current_flag = false;
        control_data.space_vector_active = SpaceVectorState::ACTIVE;
        SpeedControl::reset_PI();
        CurrentControl::reset_PI();
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        CurrentControl::set_current_ref(Comms::current_reference_received);
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_frequency_Modulation(Comms::frequency_space_vector_received);
        SpaceVector::reset_time();

        SpeedControl::stop();
        CurrentControl::start();
    }

    if(OrderPackets::Send_Reference_Speed_flag==true)
    {
        OrderPackets::Send_Reference_Speed_flag=false;
        control_data.space_vector_active = SpaceVectorState::ACTIVE;
        SpeedControl::reset_PI();
        CurrentControl::reset_PI();
        SpeedControl::set_reference_speed(Comms::speed_reference_received);
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        SpaceVector::reset_time();

        CurrentControl::start();
        SpeedControl::start();

    }

    if(OrderPackets::Reset_Bypass_flag==true)
    {
        OrderPackets::Reset_Bypass_flag=false;
        if(Comms::reset_bypass_received==false)
            Actuators::disable_reset_bypass();
        else
        Actuators::enable_reset_bypass();
    }

    if(OrderPackets::Zeroing_flag==true)
    {
        OrderPackets::Zeroing_flag=false;
        CurrentSensors::zeroing();
    }

    // Control updates:
    if(flag_update_speed_control)
    {
        flag_update_speed_control=false;
        SpeedControl::control_action();
    }

    if(flag_update_current_control)
    {
        flag_update_current_control=false;
        CurrentControl::control_action();
    }

    if(flag_execute_space_vector_control)
    {
        flag_execute_space_vector_control=false;
        SpaceVector::calculate_duties();
    }
}
