#pragma once
#include "PCU/Data/Data.hpp"
static constexpr uint32_t initial_frequency = 10000;
static constexpr std::chrono::nanoseconds dead_time_ns(300);

class PWMActuators{

    private:
    //por cambiar a tiempo de compilacion;
        inline static DualPWM U_Dual{Pinout::U_PWM, Pinout::U_PWM_NEGATED};
        inline static DualPWM V_Dual{Pinout::V_PWM, Pinout::V_PWM_NEGATED};
        inline static DualPWM W_Dual{Pinout::W_PWM, Pinout::W_PWM_NEGATED};
    public:
        PWMActuators()=default;

    static void init(){
        U_Dual.set_frequency(initial_frequency);
        V_Dual.set_frequency(initial_frequency);
        W_Dual.set_frequency(initial_frequency);

        U_Dual.set_dead_time(dead_time_ns);
        V_Dual.set_dead_time(dead_time_ns);
        W_Dual.set_dead_time(dead_time_ns);

        U_Dual.set_duty_cycle(0.0f);
        V_Dual.set_duty_cycle(0.0f);
        W_Dual.set_duty_cycle(0.0f);

        //Datos y tal por poner:
    }

    static void stop(){
        U_Dual.set_duty_cycle(0.0f);
        V_Dual.set_duty_cycle(0.0f);
        W_Dual.set_duty_cycle(0.0f);
    }


};