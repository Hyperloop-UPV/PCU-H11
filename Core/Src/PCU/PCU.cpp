#include "PCU/PCU.hpp"
static constexpr string PCU_IP = "192.168.1.5";
static const string PCU_MAC = "00:80:e1:15:00:00";
static const string PCU_MASK= "255.255.0.0";

void PCU::start()
{
    STLIB::start(PCU_MAC,PCU_IP,PCU_MASK);
    Scheduler::start();


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

void PCU::start_precharge()
{

}

void PCU::motor_brake()
{
    
}

void PCU::update()
{
    if(OrderPackets::Stop_Motor_flag == true)
    {
        OrderPackets::Stop_Motor_flag=false;
        stop_motors();
    }
    else if(OrderPackets::Start_SVPWM_flag == true)
    {
        OrderPackets::Start_SVPWM_flag=false;
        control_data.space_vector_active=true;
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_frequency_Modulation(Comms::frequency_space_vector_received);
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        SpaceVector::set_target_voltage(Comms::ref_voltage_space_vector_received);
        CurrentControl::stop();
        SpeedControl::stop();

    }

    if(OrderPackets::Send_Reference_Current_flag == true)
    {
        OrderPackets::Send_Reference_Current_flag = false;
        SpeedControl::reset_PI();
        SpaceVector::set_VMAX(237); //Ni idea por que hacen esto
        CurrentControl::set_current_ref(62.5f);
        PWMActuators::set_three_frequencies(20000);
        SpaceVector::set_frequency_Modulation(8.3);

        control_data.space_vector_active = true;
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
        control_data.space_vector_active = true;
        CurrentControl::start();
        SpeedControl::start();

    }
}