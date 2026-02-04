#include "PCU/Comms/Comms.hpp"
#include "PCU/PCU.hpp"

void Comms::start()
{
    DataPackets::pwm_packet_init(
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

    DataPackets::Current_sensors_init(
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

    DataPackets::StateMachine_states_init(
        PCU::current_state_pcu,
        PCU::current_operational_state_pcu,
        PCU::control_data.space_vector_active,
        PCU::control_data.current_control_active,
        PCU::control_data.speed_control_active
    );

    DataPackets::Speetec_data_init(
        PCU::control_data.position_encoder,
        PCU::control_data.established_direction,
        Speetec::speed_encoder,
        PCU::control_data.speed_km_h_encoder,
        Speetec::acceleration_encoder
    );

    DataPackets::Speed_data_init(
        PCU::control_data.target_speed,
        PCU::control_data.speed_error,
        PCU::control_data.actual_current_ref
    );

    DataPackets::GateDriverReporting_init(
        Sensors::gd_fault_a,
        Sensors::gd_fault_b,
        Sensors::gd_ready_a,
        Sensors::gd_ready_b
    );
    DataPackets::start();

    OrderPackets::Reset_Bypass_init(reset_bypass_received);

    OrderPackets::Send_Reference_Current_init(
        frequency_space_vector_received,
        frequency_received,
        current_reference_received,
        Vmax_control_received
    );

    OrderPackets::Send_Reference_Speed_init(
        speed_reference_received,
        frequency_received,
        Vmax_control_received
    );

    OrderPackets::Start_SVPWM_init(
        frequency_space_vector_received,
        frequency_received,
        Vmax_control_received,
        Vmax_control_received
    );

    OrderPackets::Zeroing_init();
    OrderPackets::Stop_Motor_init();

}

