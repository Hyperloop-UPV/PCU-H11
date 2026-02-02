#include "PCU/Control/Runs.hpp"
#include "PCU/PCU.hpp" 

void RUNS::init()
{
    PCU::control_data.state_run = RunState::NOTHING;
    PCU::control_data.received_stop_motor = false;
    PCU::control_data.received_motor_brake_order = false;
    vel_ref = 0.0;
}

float RUNS::update()
{
    double vel = PCU::control_data.speed_km_h_encoder;
    double pos = PCU::control_data.position_encoder;

    if(PCU::control_data.state_run == RunState::BRAKING)
    {
        if(vel <= 0.1)
        {
            PCU::control_data.received_stop_motor = true;
        }
        return 0.0;
    }

    vel_ref = 35.0F;

    if(pos > BRAKING_DISTANCE)
    {
        PCU::control_data.received_motor_brake_order = true;
        PCU::control_data.state_run = RunState::BRAKING;
        vel_ref = 0.0;
    }
    
    if(vel > vel_ref)
    {
        vel_ref = 0.0;
    }

    return vel_ref;
}

void RUNS::start(uint8_t run_id)
{
    PCU::control_data.run_mode = (RunMode)run_id;
    PCU::control_data.state_run = RunState::MOVING;
    PCU::control_data.received_stop_motor = false;
    PCU::control_data.received_motor_brake_order = false;
}