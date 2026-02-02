#pragma once
#include "PCU/Data/Data.hpp"
#include "PCU/Control/SpaceVector.hpp"
#include "PCU/Control/CurrentControl.hpp"

class SpeedControl{
private:
    inline static float reference_speed{};
    inline static PI<IntegratorType::Trapezoidal> speed_PI{Speed_Control_Data::kp_accelerate,Speed_Control_Data::ki_accelerate,Speed_Control_Data::period};
    inline static PI<IntegratorType::Trapezoidal> regenerate_PI{Speed_Control_Data::kp_regenerate,Speed_Control_Data::ki_regenerate,Speed_Control_Data::period};

public:
    inline static bool running{false};
    inline static float current_ref{};
    SpeedControl()=default;
    static void init();
    static void set_reference_speed(float speed_ref);
    static float get_reference_speed();
    static ControlStates get_controlState();
    static void change_mode(ControlStates state);
    static void control_action();
    static void reset_PI();
    
    static void start();
    static void stop();
};