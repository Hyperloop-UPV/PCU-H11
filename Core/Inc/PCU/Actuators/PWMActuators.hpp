#pragma once
#include "PCU/Data/Data.hpp"



class PWMActuators{
;
    private:
        inline static ST_LIB::DualPWM<Pinout::tim_decl,Pinout::U_PWM_pin,Pinout::U_PWM_negated_pin>* U_Dual {nullptr};
        inline static ST_LIB::DualPWM<Pinout::tim_decl,Pinout::V_PWM_pin,Pinout::V_PWM_negated_pin>* V_Dual {nullptr};
        inline static ST_LIB::DualPWM<Pinout::tim_decl,Pinout::W_PWM_pin,Pinout::W_PWM_negated_pin>* W_Dual {nullptr};

    public:
        PWMActuators()=default;

    static void init(ST_LIB::DualPWM<Pinout::tim_decl,Pinout::U_PWM_pin,Pinout::U_PWM_negated_pin>& pwm_u,
                     ST_LIB::DualPWM<Pinout::tim_decl,Pinout::V_PWM_pin,Pinout::V_PWM_negated_pin>& pwm_v,
                     ST_LIB::DualPWM<Pinout::tim_decl,Pinout::W_PWM_pin,Pinout::W_PWM_negated_pin>& pwm_w);

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