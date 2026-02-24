#include "PCU/PCU.hpp"

void PCU::start()
{
    Comms::start();
    CurrentControl::init();
    SpeedControl::init();

    Scheduler::register_task(1000, [](){flag_sensors_update = true;});

    control_data.space_vector_active = SpaceVectorState::DISABLE;
    control_data.speed_control_active = SpeedControlState::DISABLE;
    control_data.current_control_active = CurrentControlState::DISABLE;
    initialize_protections();

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
    ProtectionManager::initialize();

    ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_u_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_v_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_w_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});

    Scheduler::register_task(1000, [](){
        ProtectionManager::check_protections();
    });
    ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_u_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});
    
    ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_v_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});
    
    ProtectionManager::_add_protection(
    &CurrentSensors::actual_current_sensor_w_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});
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
    PCU_State_Machine.check_transitions();
    current_state_pcu = PCU_State_Machine.get_current_state();
    current_operational_state_pcu = Operational_State_Machine.get_current_state();

    [[maybe_unused]]static SpeedControlState speed_control_active= control_data.speed_control_active;
    [[maybe_unused]]static CurrentControlState current_control_active= control_data.current_control_active;
    [[maybe_unused]]static SpaceVectorState space_vector_active = control_data.space_vector_active;

    if(current_state_pcu == States_PCU::Fault)
    {
        return;
    }
    

    if(OrderPackets::Stop_Motor_flag == true)
    {
        OrderPackets::Stop_Motor_flag=false;
        stop_motors();
        control_data.space_vector_active = SpaceVectorState::DISABLE;
    }
    //Temporary protections
    if(current_operational_state_pcu == Operational_States_PCU::Accelerating && (CurrentSensors::actual_current_sensor_u_a >=110.0f || CurrentSensors::actual_current_sensor_v_a>=110.0f || CurrentSensors::actual_current_sensor_w_a >=110.0f)){
        PCU_State_Machine.force_change_state(fault_state); return;
    }
    
    if(OrderPackets::Start_SVPWM_flag == true)
    {
        OrderPackets::Start_SVPWM_flag=false;
        control_data.space_vector_active=SpaceVectorState::ACTIVE;
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_frequency_Modulation(Comms::frequency_space_vector_received);
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        SpaceVector::set_target_voltage(Comms::ref_voltage_space_vector_received);
        CurrentControl::stop();
        SpeedControl::stop();

    }

    //Arreglar esto:

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

    if(flag_sensors_update)
    {
        flag_sensors_update=false;
        Sensors::read();
        VoltageSensors::read();
        CurrentSensors::read();
        Speetec::read();

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
