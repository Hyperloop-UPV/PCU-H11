#include "PCU/Actuators/PWMActuators.hpp"

static constexpr uint32_t initial_frequency = 10000;
static constexpr std::chrono::nanoseconds dead_time_ns(300);

PWM_ACTIVE PWMActuators::active_pwm = PWM_ACTIVE::NONE;

void PWMActuators::init()
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
float PWMActuators::get_duty_u(){return U_Dual.get_duty_cycle();}
float PWMActuators::get_duty_v(){return V_Dual.get_duty_cycle();}
float PWMActuators::get_duty_w(){return W_Dual.get_duty_cycle();}
float PWMActuators::get_frequency_u(){return U_Dual.get_frequency();}
float PWMActuators::get_frequency_v(){return V_Dual.get_frequency();}
float PWMActuators::get_frequency_w(){return W_Dual.get_frequency();}


/*------Switching PWM------*/
void PWMActuators::stop()
{
    U_Dual.set_duty_cycle(0.0f);
    V_Dual.set_duty_cycle(0.0f);
    W_Dual.set_duty_cycle(0.0f);

    active_pwm = PWM_ACTIVE::NONE;
    current_duty_cycle = 0.0f;
    current_frequency = 0;
}

void PWMActuators::turn_off_u()
{
    U_Dual.set_duty_cycle(0.0f);
    active_pwm = PWM_ACTIVE::NONE;
    current_duty_cycle = 0.0f;
    current_frequency = 0;
}
void PWMActuators::turn_off_w()
{
    W_Dual.set_duty_cycle(0.0f);
    active_pwm = PWM_ACTIVE::NONE;
    current_duty_cycle = 0.0f;
    current_frequency = 0;
}
void PWMActuators::turn_off_v()
{
    V_Dual.set_duty_cycle(0.0f);
    active_pwm = PWM_ACTIVE::NONE;
    current_duty_cycle = 0.0f;
    current_frequency = 0;
}

void PWMActuators::turn_off_active_pwm() 
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
void PWMActuators::set_duty_u(float duty_cycle)
{
    if (duty_cycle < 0.0)
        duty_cycle = 0.0;
    else if (duty_cycle > 100.0)
        duty_cycle = 100.0;

    U_Dual.set_duty_cycle(duty_cycle);
    current_duty_cycle = get_duty_u();
    active_pwm = PWM_ACTIVE::U;
}
void PWMActuators::set_frequency_u(uint32_t frequency)
{
    U_Dual.set_frequency(frequency);
    current_frequency = get_frequency_u();
}



void PWMActuators::set_duty_v(float duty_cycle)
{
    if (duty_cycle < 0.0)
        duty_cycle = 0.0;
    else if (duty_cycle > 100.0)
        duty_cycle = 100.0;

    V_Dual.set_duty_cycle(duty_cycle);
    current_duty_cycle = get_duty_v();
    active_pwm = PWM_ACTIVE::V;
}
void PWMActuators::set_frequency_v(uint32_t frequency)
{
    V_Dual.set_frequency(frequency);
    current_frequency = get_frequency_v();
}



void PWMActuators::set_duty_w(float duty_cycle)
{
    if (duty_cycle < 0.0)
        duty_cycle = 0.0;
    else if (duty_cycle > 100.0)
        duty_cycle = 100.0;
        
    W_Dual.set_duty_cycle(duty_cycle);
    current_duty_cycle = get_duty_w();
    active_pwm = PWM_ACTIVE::W;
}
void PWMActuators::set_frequency_w(uint32_t frequency)
{
    W_Dual.set_frequency(frequency);
    current_frequency = get_frequency_w();
}



void PWMActuators::set_three_frequencies(uint32_t frequency) 
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
