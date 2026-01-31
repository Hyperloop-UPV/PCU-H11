#pragma once
#include "PCU/Data/Data.hpp"
static constexpr uint32_t initial_frequency = 10000;
static constexpr std::chrono::nanoseconds dead_time_ns(300);

enum class PWM_ACTIVE: uint8_t
{
    NONE = 0,
    U = 1,
    V = 2,
    W = 3
}; 
class PWMActuators{

    private:
    //por cambiar a tiempo de compilacion;
        inline static DualPWM U_Dual{Pinout::U_PWM, Pinout::U_PWM_NEGATED};
        inline static DualPWM V_Dual{Pinout::V_PWM, Pinout::V_PWM_NEGATED};
        inline static DualPWM W_Dual{Pinout::W_PWM, Pinout::W_PWM_NEGATED};

    public:
        static PWM_ACTIVE active_pwm;
        inline static uint32_t current_frequency{0};
        inline static float current_duty_cycle{0.0f};
        PWMActuators()=default;

    static void init()
    {
        U_Dual.set_frequency(initial_frequency);
        V_Dual.set_frequency(initial_frequency);
        W_Dual.set_frequency(initial_frequency);

        U_Dual.set_dead_time(dead_time_ns);
        V_Dual.set_dead_time(dead_time_ns);
        W_Dual.set_dead_time(dead_time_ns);

        U_Dual.set_duty_cycle(0.0f);
        V_Dual.set_duty_cycle(0.0f);
        W_Dual.set_duty_cycle(0.0f);

        active_pwm = PWM_ACTIVE::NONE;
    }



    /*------Getters PWM------*/
    static float get_duty_u(){return U_Dual.get_duty_cycle();}
    static float get_duty_v(){return V_Dual.get_duty_cycle();}
    static float get_duty_w(){return W_Dual.get_duty_cycle();}
    static float get_frequency_u(){return U_Dual.get_frequency();}
    static float get_frequency_v(){return V_Dual.get_frequency();}
    static float get_frequency_w(){return W_Dual.get_frequency();}


    /*------Switching PWM------*/
    static void stop()
    {
        U_Dual.set_duty_cycle(0.0f);
        V_Dual.set_duty_cycle(0.0f);
        W_Dual.set_duty_cycle(0.0f);

        active_pwm = PWM_ACTIVE::NONE;
        current_duty_cycle = 0.0f;
        current_frequency = 0;
    }

    static void turn_off_u()
    {
        U_Dual.set_duty_cycle(0.0f);
        active_pwm = PWM_ACTIVE::NONE;
        current_duty_cycle = 0.0f;
        current_frequency = 0;
    }
    static void turn_off_w()
    {
        W_Dual.set_duty_cycle(0.0f);
        active_pwm = PWM_ACTIVE::NONE;
        current_duty_cycle = 0.0f;
        current_frequency = 0;
    }
    static void turn_off_v()
    {
        V_Dual.set_duty_cycle(0.0f);
        active_pwm = PWM_ACTIVE::NONE;
        current_duty_cycle = 0.0f;
        current_frequency = 0;
    }

    static void turn_off_active_pwm() 
    {
    switch (active_pwm) {
        case PWM_ACTIVE::NONE:
            return;
        case PWM_ACTIVE::U:
            turn_off_u();
            return;
        case PWM_ACTIVE::V:
            turn_off_v();
            return;
        case PWM_ACTIVE::W:
            turn_off_w();
    }
    }


    /*------Setters PWM------*/
    static void set_duty_u(float duty_cycle)
    {
        U_Dual.set_duty_cycle(duty_cycle);
        current_duty_cycle = get_duty_u();
        active_pwm = PWM_ACTIVE::U;
    }
    static void set_frequency_u(uint32_t frequency)
    {
        U_Dual.set_frequency(frequency);
        current_frequency = get_frequency_u();
    }



    static void set_duty_v(float duty_cycle)
    {
        V_Dual.set_duty_cycle(duty_cycle);
        current_duty_cycle = get_duty_v();
        active_pwm = PWM_ACTIVE::V;
    }
    static void set_frequency_v(uint32_t frequency)
    {
        V_Dual.set_frequency(frequency);
        current_frequency = get_frequency_v();
    }



    static void set_duty_w(float duty_cycle)
    {
        W_Dual.set_duty_cycle(duty_cycle);
        current_duty_cycle = get_duty_w();
        active_pwm = PWM_ACTIVE::W;
    }
    static void set_frequency_w(uint32_t frequency)
    {
        W_Dual.set_frequency(frequency);
        current_frequency = get_frequency_w();
    }

    static void set_three_frequencies(uint32_t frequency) 
    {
    U_Dual.set_frequency(frequency);
    V_Dual.set_frequency(frequency);
    W_Dual.set_frequency(frequency);
    
    if(frequency != get_frequency_u()&& frequency != get_frequency_v()&& frequency != get_frequency_w())
    {
        //manejar error??
    }
    current_frequency = frequency;
}



};