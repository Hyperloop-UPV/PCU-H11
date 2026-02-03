#pragma once
#include "PCU/Data/Data.hpp"

class Comms{
    Comms()=default;
    public:
    inline static float frequency_received{0.0f};
    inline static float duty_cycle_received{0.0f};
    inline static float frequency_space_vector_received{0.0f};
    inline static float ref_voltage_space_vector_received{0.0f};
    inline static float current_reference_received{0.0f};
    inline static float Vmax_control_received{0.0f};
    inline static float speed_reference_received{0.0f};

    static void start();
    static void update();

};