#pragma once
#include "PCU/Actuators/Actuators.hpp"
#include "PCU/Actuators/PWMActuators.hpp"
#include "PCU/Data/Data.hpp"

#if MODE_CALCULATE_SIN == 1
#include "PCU/Control/LookUpTable.hpp"
enum phase
{
    U,
    V,
    W
};
#endif

class SpaceVector
{
private:

    inline static float time{0.0f};
    static float Imodulation;
    static double Modulation_frequency;

    static constexpr float IMAX = 1.1547;
    
public:

    static constexpr uint32_t Period = 200; // this Period is in microseconds
    static inline float VMAX = 163.0;

    SpaceVector()=default;
    static void set_target_voltage(float V_ref);
    static void set_frequency_Modulation(float freq);
    static void calculate_duties();
    static float get_modulation_frequency();
    static float get_actual_time();
    static void set_VMAX(float VMax);
    #if MODE_CALCULATE_SIN == 1
        static float calculate_sin_look_up_table(float angle);
        static float calculate_sin_phase(phase p);
    #endif
};