#include "PCU/PCU.hpp"
static constexpr string PCU_IP = "192.168.1.5";
static const string PCU_MAC = "00:80:e1:15:00:00";
static const string PCU_MASK= "255.255.0.0";

void PCU::start()
{
    STLIB::start(PCU_MAC,PCU_IP,PCU_MASK); //POr cambiar a lo nuevo
    Comms::start();

}

void PCU::stop_motors()
{
    flag_update_current_control = false;
    flag_update_speed_control = false;
    CurrentControl::stop();
    SpeedControl::stop();
    PWMActuators::stop();
    Actuators::disable_buffer();
}

void PCU::update()
{
    PCU_State_Machine.check_transitions();

    if(OrderPackets::Stop_Motor_flag == true)
    {
        OrderPackets::Stop_Motor_flag=false;
        stop_motors();
    }
    current_state_pcu = PCU_State_Machine.current_state;
    current_operational_state_pcu = Operational_State_Machine.current_state;
    
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
        SpeedControl::reset_PI();
        SpaceVector::set_VMAX(237); //Ni idea por que hacen esto
        CurrentControl::set_current_ref(62.5f);
        PWMActuators::set_three_frequencies(20000);
        SpaceVector::set_frequency_Modulation(8.3);

        control_data.space_vector_active = SpaceVectorState::ACTIVE;
        SpeedControl::stop();
        CurrentControl::start();
    }

    if(OrderPackets::Send_Reference_Speed_flag==true)
    {
        OrderPackets::Send_Reference_Speed_flag=false;
        SpeedControl::reset_PI();
        SpeedControl::set_reference_speed(Comms::speed_reference_received);
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        control_data.space_vector_active = SpaceVectorState::ACTIVE;
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

    //Control updates:
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