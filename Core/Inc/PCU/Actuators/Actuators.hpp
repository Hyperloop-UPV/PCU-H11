#pragma once
#include "PCU/Data/Data.hpp"
#include "PCU/PCU.hpp"




class Actuators
{
    private:
    struct NegatedPin
    {
        ST_LIB::DigitalOutputDomain::Instance& pin;
        void turn_on(){pin.turn_off();}
        void turn_off(){pin.turn_on();}
    };

    using DigitalOutputInstance = ST_LIB::DigitalOutputDomain::Instance;

    /*------Control outputs-------*/
    #if PCU_H10 == 0
    static DigitalOutputInstance& speedtec_supply;
    static DigitalOutputInstance& hall_supply_a;
    static DigitalOutputInstance& hall_supply_b;
    #endif
    static NegatedPin enable_pin;
    static DigitalOutputInstance& reset_bypass;

    /*-----Leds----*/
    static DigitalOutputInstance& led_operational;
    static DigitalOutputInstance& led_fault;
    static DigitalOutputInstance& led_connecting;

    #if PCU_H10 == 0
    static DigitalOutputInstance& led_accelerating;
    static DigitalOutputInstance& led_braking;
    #endif

    public:
    Actuators()=default;

    #if PCU_H10 == 0
    static void init(DigitalOutputInstance& hall_supply_a_instance,
                     DigitalOutputInstance& hall_supply_b_instance,
                     DigitalOutputInstance& speedtec_supply_instance,
                     DigitalOutputInstance& enable_pin_instance,
                     DigitalOutputInstance& reset_pin_instance);

    static void init_leds(DigitalOutputInstance& led_operational_instance,
                          DigitalOutputInstance& led_fault_instance,
                          DigitalOutputInstance& led_connecting_instance,
                          DigitalOutputInstance& led_accelerating_instance,
                          DigitalOutputInstance& led_braking_instance)
    {
        led_operational = led_operational_instance;
        led_fault = led_fault_instance;
        led_connecting = led_connecting_instance;
        led_accelerating = led_accelerating_instance;
        led_braking = led_braking_instance;

        led_operational.turn_off();
        led_fault.turn_off();
        led_connecting.turn_off();
        led_accelerating.turn_off();
        led_braking.turn_off();
    }
    #else
    static void init(DigitalOutputInstance& enable_pin_instance,
                     DigitalOutputInstance& reset_pin_instance,
                     DigitalOutputInstance& led_operational_instance,
                     DigitalOutputInstance& led_fault_instance,
                     DigitalOutputInstance& led_connecting_instance);
    #endif

    static void enable_buffer();
    static void disable_buffer();

    static void enable_reset_bypass();
    static void disable_reset_bypass();

    #if PCU_H10 == 0
    static void enable_hall_supply();
    static void disable_hall_supply();
    static void enable_speedtec_supply();
    static void disable_speedtec_supply();
    #endif

    static void set_led_operational(bool state);
    static void set_led_fault(bool state);
    static void set_led_connecting(bool state);

    #if PCU_H10 == 0
    static void set_led_accelerating(bool state);
    static void set_led_braking(bool state);
    #endif

};