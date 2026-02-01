#include "PCU/Control/SpeedControl.hpp"
#include "PCU/PCU.hpp"

#define CURRENT_LIMIT 100
#define REGENERATIVE_SPEED_REF 0.0

static double exp_follower(double reference, double error_factor = 0.5){
    static double output{0.0};
    double error{reference - output};
    output += error * error_factor;
    return output;
}
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
double SpeedControl::calculate_frequency_modulation(){
    return (PCU::control_data.speedState == ControlStates::accelerate) ? exp_follower(a*PCU::control_data.speed_km_h_encoder + b) : exp_follower(a*PCU::control_data.speed_km_h_encoder + b - PCU::control_data.speed_km_h_encoder/1.2);
}
void SpeedControl::control_action(){
    if(!CurrentControl::is_running())  return;
    SpaceVector::set_frequency_Modulation(calculate_frequency_modulation());
    double speed_error = reference_speed - PCU::control_data.speed_km_h_encoder;
    PCU::control_data.speed_error = speed_error;
    float actual_current_ref;
    if(PCU::control_data.speedState == ControlStates::accelerate){
        speed_PI.input(speed_error);
        speed_PI.execute(); 
        actual_current_ref = speed_PI.output_value;
    }else{
        regenerate_PI.input(speed_error);
        regenerate_PI.execute();
        actual_current_ref = regenerate_PI.output_value;
    }
    actual_current_ref = (actual_current_ref > CURRENT_LIMIT || actual_current_ref < -30.0) ? CURRENT_LIMIT : actual_current_ref;
    /*if(actual_current_ref > CURRENT_LIMIT)
        actual_current_ref = CURRENT_LIMIT;
    if(actual_current_ref <= 0.0){
        actual_current_ref = 0.0;
    }*/
        
   PCU::control_data.actual_current_ref = actual_current_ref;
    CurrentControl::set_current_ref(actual_current_ref);
    //if we are in regenerate and we arrive to the max speed we change the reference speed to zero
   /* if(PCU::control_data.speedState == ControlStates::regenerate && reference_speed > 0.1 &&  PCU::control_data.speed_km_h_encoder > reference_speed){
        set_reference_speed(REGENERATIVE_SPEED_REF);
    }*/
}
ControlStates SpeedControl::get_controlState(){
    return PCU::control_data.speedState;
}
void SpeedControl::change_mode(ControlStates state){
    PCU::control_data.speedState = state;
}
void SpeedControl::reset_PI(){
    speed_PI.reset();
    regenerate_PI.reset();
    CurrentControl::reset_PI();
}