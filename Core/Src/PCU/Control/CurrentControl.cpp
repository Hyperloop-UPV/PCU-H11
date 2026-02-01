#include "PCU/Control/CurrentControl.hpp"
#include "PCU/PCU.hpp"

double Max_Peak::modulation_frequency = 0.0;

void CurrentControl::set_current_ref(float cur_ref){
    current_ref = cur_ref;
}

float CurrentControl::get_current_ref(){
    return current_ref;
}
double CurrentControl::calculate_peak(){
    Max_Peak::set_modulation_freq(SpaceVector::get_modulation_frequency());
    #if PPU_USING != 1
        double Peak_u_a = current_u_a.calculate_Max_Peak();
        double Peak_v_a = current_v_a.calculate_Max_Peak();
        double Peak_w_a = current_w_a.calculate_Max_Peak();
        double average_a = (Peak_u_a + Peak_v_a + Peak_w_a)/3.0;
    #endif
    #if PPU_USING != 0
        double Peak_u_b = current_u_b.calculate_Max_Peak();
        double Peak_v_b = current_v_b.calculate_Max_Peak();
        double Peak_w_b = current_w_b.calculate_Max_Peak();
        double average_b = (Peak_u_b + Peak_v_b + Peak_w_b)/3.0;
    #endif
    #if PPU_USING == 0
        return average_a;
    #elif PPU_USING == 1
        return average_b;
    #elif PPU_USING == 2
        return (average_a + average_b)/2.0;
    #endif
}
void CurrentControl::control_action(){
    if (!should_be_running) return;
    double target_voltage;
    double current_peak = calculate_peak();
    double current_error = current_ref - current_peak;
    PCU::control_data.current_Peak = current_peak;
    PCU::control_data.current_error = current_error;
    
    if(PCU::control_data.currentState == ControlStates::accelerate){
        #if SATURATOR_PI
        float integrator_temp = current_PI.integrator.output_value;
        #endif
        current_PI.input(current_error);
        current_PI.execute();
        #if SATURATOR_PI
        target_voltage = current_PI.output_value;
        if(current_ref <= 0.0){
            current_PI.integrator.output_value = integrator_temp;
        }
        #endif
    }
    else{
        current_regenerate_PI.input(current_error);
        current_regenerate_PI.execute();
        target_voltage = current_regenerate_PI.output_value;
    }

    PCU::control_data.target_voltage = (target_voltage <= SpaceVector::VMAX) ? 
                            (target_voltage < 0.0) ? 0.0 : target_voltage 
                            : SpaceVector::VMAX;
    SpaceVector::set_target_voltage(PCU::control_data.target_voltage);
}

void CurrentControl::start() {
    should_be_running = true;
    reset_PI();
}

void CurrentControl::stop() {
    should_be_running = false;
}
bool CurrentControl::is_running(){
    return should_be_running;
}
void CurrentControl::change_mode(ControlStates state){
    PCU::control_data.currentState = state;
}
void CurrentControl::reset_PI(){
    current_PI.reset();
    current_regenerate_PI.reset();
}