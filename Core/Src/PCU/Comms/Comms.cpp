#include "PCU/Comms/Comms.hpp"
#include "PCU/PCU.hpp"

void Comms::start()
{
    DataPackets::PWM_init(
        PCU::control_data.actual_frequency,
        PCU::control_data.modulation_frequency,
        PCU::control_data.duty_cycle_u,
        PCU::control_data.duty_cycle_v,
        PCU::control_data.duty_cycle_w
    );

    DataPackets::Batteries_Voltage_init(
        VoltageSensors::actual_voltage_battery_a,
        VoltageSensors::actual_voltage_battery_b
    );

    DataPackets::Current_Sensors_init(
        CurrentSensors::actual_current_sensor_u_a,
        CurrentSensors::actual_current_sensor_v_a,
        CurrentSensors::actual_current_sensor_w_a,
        CurrentSensors::actual_current_sensor_u_b,
        CurrentSensors::actual_current_sensor_v_b,
        CurrentSensors::actual_current_sensor_w_b,
        PCU::control_data.current_Peak,
        PCU::control_data.current_error,
        PCU::control_data.target_voltage,
        PCU::control_data.time,
        PCU::control_data.imod
    );

    DataPackets::State_Machine_States_init(
        PCU::current_state_pcu
    );

    DataPackets::Speed_Data_init(
        PCU::control_data.target_speed,
        PCU::control_data.speed_error,
        PCU::control_data.actual_current_ref,
        PCU::control_data.slip_control,
        PCU::control_data.IMU_speed_km_h,
        PCU::control_data.IMU_position_m
    );

    DataPackets::Gate_Driver_Reporting_init(
        Sensors::gd_fault_a,
        Sensors::gd_fault_b,
        Sensors::gd_ready_a,
        Sensors::gd_ready_b
    );
    DataPackets::start();

    OrderPackets::Send_Reference_Current_init(
        Comms::frequency_space_vector_received,
        Comms::frequency_received,
        Comms::current_reference_received,
        Comms::Vmax_control_received,
        Comms::Variable_frequency_recieved,
        Comms::Reverse_direction
    );

    OrderPackets::Send_Reference_Speed_init(
        Comms::speed_reference_received,
        Comms::frequency_received,
        Comms::Vmax_control_received,
        Comms::Reverse_direction
    );

    OrderPackets::Start_SVPWM_init(
        Comms::frequency_space_vector_received,
        Comms::frequency_received,
        Comms::ref_voltage_space_vector_received,
        Comms::Vmax_control_received
    );

    OrderPackets::Zeroing_init();
    OrderPackets::Stop_Motor_init();

    OrderPackets::start();
    Diagnostics::install_ethernet_sink(OrderPackets::control_station_tcp);

}

