#include "PCU/PCU.hpp"

void PCU::start()
{
    CurrentControl::init();
    SpeedControl::init();
    PCU_State_Machine.start();

    Scheduler::register_task(Sensors_data::read_sensors_us, [](){
        Speetec::read();
        control_data.cnt_encoder = TIM23->CNT;
    });

    Scheduler::register_task(400, [](){
        CurrentSensors::read();
    });

    Scheduler::register_task(1000, [](){
        static double aux_speed_IMU = 0;
        aux_speed_IMU = IMU::get_imu_x_speed();
        __disable_irq();
        control_data.IMU_speed_km_h = aux_speed_IMU;
        __enable_irq();
    });

    // Scheduler::register_task(1'000, [](){
    //     PCU_State_Machine.check_transitions();
    //     current_state_pcu = PCU_State_Machine.get_current_state();
    //     current_operational_state_pcu = Operational_State_Machine.get_current_state();
    //     Sensors::read();
    //     VoltageSensors::read();
    // });


    control_data.space_vector_active = SpaceVectorState::DISABLE;
    control_data.speed_control_active = SpeedControlState::DISABLE;
    control_data.current_control_active = CurrentControlState::DISABLE;
    initialize_protections();

    Scheduler::register_task(1'000, [](){
        static constexpr float max_current = CURRENT_PROTECTION;
        static constexpr float max_voltage = Protecction_Voltage;
        ProtectionManager::check_protections();
        if(VoltageSensors::actual_voltage_battery_a > Protecction_Voltage) {
            ErrorHandler("PCU Battery voltage A %f above limit: %f", VoltageSensors::actual_voltage_battery_a, max_voltage);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }

        if(VoltageSensors::actual_voltage_battery_b > Protecction_Voltage) {
            ErrorHandler("PCU Battery voltage B %f above limit: %f", VoltageSensors::actual_voltage_battery_b, max_voltage);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }

        if(CurrentSensors::actual_current_sensor_u_a > CURRENT_PROTECTION) {
            ErrorHandler("PCU Current Sensor U A %f above limit: %f", CurrentSensors::actual_current_sensor_u_a, max_current);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }
        if(CurrentSensors::actual_current_sensor_v_a > CURRENT_PROTECTION) {
            ErrorHandler("PCU Current Sensor V A %f above limit: %f", CurrentSensors::actual_current_sensor_v_a, max_current);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }
        if(CurrentSensors::actual_current_sensor_w_a > CURRENT_PROTECTION) {
            ErrorHandler("PCU Current Sensor W A %f above limit: %f", CurrentSensors::actual_current_sensor_w_a, max_current);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }
        
        if(CurrentSensors::actual_current_sensor_u_b > CURRENT_PROTECTION) {
            ErrorHandler("PCU Current Sensor U B %f above limit: %f", CurrentSensors::actual_current_sensor_u_b, max_current);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }
        if(CurrentSensors::actual_current_sensor_v_b > CURRENT_PROTECTION) {
            ErrorHandler("PCU Current Sensor V B %f above limit: %f", CurrentSensors::actual_current_sensor_v_b, max_current);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }
        if(CurrentSensors::actual_current_sensor_w_b > CURRENT_PROTECTION) {
            ErrorHandler("PCU Current Sensor W B %f above limit: %f", CurrentSensors::actual_current_sensor_w_b, max_current);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }

        if(control_data.position_encoder > 25.0) {
            ErrorHandler("PCU Position Encoder %f above limit: %f", control_data.position_encoder, 25.0);
            PCU_State_Machine.force_change_state(static_cast<size_t>(States_PCU::Fault));
        }
        PCU_State_Machine.check_transitions();
        current_state_pcu = PCU_State_Machine.get_current_state();
        current_operational_state_pcu = Operational_State_Machine.get_current_state();
        Sensors::read();
        VoltageSensors::read();
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

//    auto& voltage_protection_A_PCU = ProtectionManager::_add_protection(
//    &VoltageSensors::actual_voltage_battery_a, Boundary<float, ABOVE>{Protecction_Voltage});
//
//    voltage_protection_A_PCU.set_name((char*)"PCU Battery A voltage");
//
//    auto& voltage_protection_B_PCU = ProtectionManager::_add_protection(
//    &VoltageSensors::actual_voltage_battery_b, Boundary<float, ABOVE>{Protecction_Voltage});
//
//    voltage_protection_B_PCU.set_name((char*)"PCU Battery B voltage");
//
//    auto& current_protection_u_a = ProtectionManager::_add_protection(
//    &CurrentSensors::actual_current_sensor_u_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});
//
//    current_protection_u_a.set_name((char*)"PCU Current Sensor U A");
//
//    auto& current_protection_v_a = ProtectionManager::_add_protection(
//    &CurrentSensors::actual_current_sensor_v_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});
//
//    current_protection_v_a.set_name((char*)"PCU Current Sensor V A");
//    
//    auto& current_protection_w_a = ProtectionManager::_add_protection(
//    &CurrentSensors::actual_current_sensor_w_a, Boundary<float, ABOVE>{CURRENT_PROTECTION});
//
//    current_protection_w_a.set_name((char*)"PCU Current Sensor W A");
//
//    auto& current_protection_u_b = ProtectionManager::_add_protection(
//    &CurrentSensors::actual_current_sensor_u_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});
//
//    current_protection_u_b.set_name((char*)"PCU Current Sensor U B");
//    
//    auto& current_protection_v_b = ProtectionManager::_add_protection(
//    &CurrentSensors::actual_current_sensor_v_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});
//
//    current_protection_v_b.set_name((char*)"PCU Current Sensor V B");
//    
//    auto& current_protection_w_b = ProtectionManager::_add_protection(
//    &CurrentSensors::actual_current_sensor_w_b, Boundary<float, ABOVE>{CURRENT_PROTECTION});
//
//    current_protection_w_b.set_name((char*)"PCU Current Sensor W B");

    
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
    if(OrderPackets::Stop_Motor_flag == true)
    {
        OrderPackets::Stop_Motor_flag=false;
        __disable_irq();
        control_data.space_vector_active = SpaceVectorState::DISABLE;
        stop_motors();
        __enable_irq();
    }
    
    if(current_state_pcu == States_PCU::Fault)
    {
        return;
    }
    
    if(OrderPackets::Start_SVPWM_flag == true)
    {
        OrderPackets::Start_SVPWM_flag=false;
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_frequency_Modulation(Comms::frequency_space_vector_received);
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        SpaceVector::set_target_voltage(Comms::ref_voltage_space_vector_received);
        SpaceVector::reset_time();

        CurrentControl::stop();
        SpeedControl::stop();
        control_data.space_vector_active=SpaceVectorState::ACTIVE;

    }

    if(OrderPackets::Send_Reference_Current_flag == true)
    {
        OrderPackets::Send_Reference_Current_flag = false;
        SpeedControl::reset_PI();
        CurrentControl::reset_PI();
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        CurrentControl::set_current_ref(Comms::current_reference_received);
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_frequency_Modulation(Comms::frequency_space_vector_received);
        SpaceVector::reset_time();

        SpeedControl::stop();
        CurrentControl::start();
        control_data.space_vector_active = SpaceVectorState::ACTIVE;

    }

    if(OrderPackets::Send_Reference_Speed_flag==true)
    {
        OrderPackets::Send_Reference_Speed_flag=false;
        SpeedControl::reset_PI();
        CurrentControl::reset_PI();
        SpeedControl::set_reference_speed(Comms::speed_reference_received);
        PWMActuators::set_three_frequencies(Comms::frequency_received);
        SpaceVector::set_VMAX(Comms::Vmax_control_received);
        SpaceVector::reset_time();

        CurrentControl::start();
        SpeedControl::start();
        control_data.space_vector_active = SpaceVectorState::ACTIVE;


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
        IMU::calibrate();
    }
}
