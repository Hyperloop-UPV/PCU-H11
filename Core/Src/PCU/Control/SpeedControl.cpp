#include "PCU/Control/SpeedControl.hpp"
#include "PCU/PCU.hpp"

#define CURRENT_LIMIT 100
#define REGENERATIVE_SPEED_REF 0.0

void SpeedControl::init(){
    speed_PI.reset();
    regenerate_PI.reset();
}

void SpeedControl::set_reference_speed(float speed_ref){
    reference_speed = speed_ref;
    PCU::control_data.target_speed = reference_speed;
}

float SpeedControl::get_reference_speed(){
    return reference_speed;
}

void SpeedControl::control_action(){
    if(!CurrentControl::is_running() || !running) return;
    

    double speed_error = reference_speed - PCU::control_data.speed_km_h_encoder;
    PCU::control_data.speed_error = speed_error;
    float actual_current_ref;
    
    // if(PCU::control_data.speedState == ControlStates::Cruise_Mode){
        speed_PI.input(speed_error);
        speed_PI.execute(); 
        actual_current_ref = speed_PI.output_value;
    // }else{
    //     regenerate_PI.input(speed_error);
    //     regenerate_PI.execute();
    //     actual_current_ref = regenerate_PI.output_value;
    // }
    
    actual_current_ref = (actual_current_ref > CURRENT_LIMIT || actual_current_ref < -30.0) ? CURRENT_LIMIT : actual_current_ref;
        
    PCU::control_data.actual_current_ref = actual_current_ref;
    CurrentControl::set_current_ref(actual_current_ref);
}


void SpeedControl::reset_PI(){
    speed_PI.reset();
    regenerate_PI.reset();
    CurrentControl::reset_PI();
}

void SpeedControl::start(){
    running = true;
    PCU::control_data.speed_control_active = SpeedControlState::ACTIVE;
}

void SpeedControl::stop(){
    running = false;
    PCU::control_data.speed_control_active = SpeedControlState::DISABLE;
}