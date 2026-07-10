#include "PCU/PCU.hpp"

ST_LIB::EthernetDomain::Instance *ethernet;

States_Shown_PCU get_shown_state_from_internal(States_PCU state, Operational_States_PCU operational_state)
{
    switch(state) {
        case States_PCU::Connecting: return States_Shown_PCU::Connecting;
        case States_PCU::Operational: {
            if(operational_state == Operational_States_PCU::IDLE) {
                return States_Shown_PCU::Idle;
            } else {
                return States_Shown_PCU::Accelerating;
            }
        }
        case States_PCU::Fault: return States_Shown_PCU::Fault;
    }
    // unknown state
    return States_Shown_PCU::Fault;
}

void PCU::start()
{
    CurrentControl::init();
    SpeedControl::init();
    PCU_State_Machine.start();

    Actuators::enable_hall_supply();
    Actuators::enable_speedtec_supply();

    // Scheduler::register_task(Sensors_data::read_sensors_us, [](){
    //     Speetec::read();
    //     control_data.cnt_encoder = TIM23->CNT;
    // });

    Scheduler::register_task(400, [](){
        CurrentSensors::read();
    });
    
    #if PCU_H10 == 1
    Scheduler::register_task(Sensors_data::read_sensors_us, [](){
        static imu_pair aux_speed_IMU = {0, 0};
        aux_speed_IMU = IMU::get_imu_x_speed();
        __disable_irq();
        control_data.IMU_speed_km_h = aux_speed_IMU.velocity_km_h;
        __enable_irq();
        control_data.IMU_position_m = aux_speed_IMU.position_m;
    });
    #endif


    control_data.space_vector_active = SpaceVectorState::DISABLE;
    control_data.speed_control_active = SpeedControlState::DISABLE;
    control_data.current_control_active = CurrentControlState::DISABLE;

    Scheduler::register_task(1'000, [](){
        PCU_State_Machine.check_transitions();
        current_state_pcu = PCU_State_Machine.get_current_state();
        current_operational_state_pcu = Operational_State_Machine.get_current_state();
        current_sending_state = get_shown_state_from_internal(current_state_pcu, current_operational_state_pcu);
        Sensors::read();
        VoltageSensors::read();
    });

    #if PCU_H10 == 0
    Actuators::enable_hall_supply();
    Actuators::enable_speedtec_supply();
    #endif

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

    if(OrderPackets::Zeroing_flag==true)
    {
        OrderPackets::Zeroing_flag=false;
        CurrentSensors::zeroing();
        #if PCU_H10 == 1
        IMU::calibrate(10000);
        #endif
    }
}
