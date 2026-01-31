#pragma once
#include "PCU/Data/Data.hpp"
#include "PCU/PCU.hpp"



class PWMActuators{

    private:
    //por cambiar a tiempo de compilacion;
        inline static DualPWM U_Dual{Pinout::U_PWM, Pinout::U_PWM_NEGATED};
        inline static DualPWM V_Dual{Pinout::V_PWM, Pinout::V_PWM_NEGATED};
        inline static DualPWM W_Dual{Pinout::W_PWM, Pinout::W_PWM_NEGATED};

    public:
        PWMActuators()=default;

    static void init();

    /*------Getters PWM------*/
    static float get_duty_u();
    static float get_duty_v();
    static float get_duty_w();
    static float get_frequency_u();
    static float get_frequency_v();
    static float get_frequency_w();


    /*------Switching PWM------*/
    static void stop();

    static void turn_off_u();
    static void turn_off_w();
    static void turn_off_v();

    static void turn_off_active_pwm();


    /*------Setters PWM------*/
    static void set_duty_u(float duty_cycle);
    static void set_frequency_u(uint32_t frequency);

    static void set_duty_v(float duty_cycle);
    static void set_frequency_v(uint32_t frequency);

    static void set_duty_w(float duty_cycle);
    static void set_frequency_w(uint32_t frequency);

    static void set_three_frequencies(uint32_t frequency);



};