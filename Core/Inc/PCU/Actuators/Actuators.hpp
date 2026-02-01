#pragma once
#include "PCU/Data/Data.hpp"




class Actuators
{
    private:
    struct NegatedPin
    {
        ST_LIB::DigitalOutputDomain::Instance* pin;
        NegatedPin() : pin(nullptr) {}
        NegatedPin(ST_LIB::DigitalOutputDomain::Instance& p) : pin(&p) {}
        void turn_on(){if(pin) pin->turn_off();}
        void turn_off(){if(pin) pin->turn_on();}
    };

    using DigitalOutputInstance = ST_LIB::DigitalOutputDomain::Instance;

    /*------Control outputs-------*/
    #if PCU_H10 == 0
    static inline DigitalOutputInstance* speedtec_supply = nullptr;
    static inline DigitalOutputInstance* hall_supply_a = nullptr;
    static inline DigitalOutputInstance* hall_supply_b = nullptr;
    #endif
    static inline NegatedPin enable_pin;
    static inline DigitalOutputInstance* reset_bypass = nullptr;

    /*-----Leds----*/
    static inline DigitalOutputInstance* led_operational = nullptr;
    static inline DigitalOutputInstance* led_fault = nullptr;
    static inline DigitalOutputInstance* led_connecting = nullptr;

    #if PCU_H10 == 0
    static inline DigitalOutputInstance* led_accelerating = nullptr;
    static inline DigitalOutputInstance* led_braking = nullptr;
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
        led_operational = &led_operational_instance;
        led_fault = &led_fault_instance;
        led_connecting = &led_connecting_instance;
        led_accelerating = &led_accelerating_instance;
        led_braking = &led_braking_instance;

        if(led_operational) led_operational->turn_off();
        if(led_fault) led_fault->turn_off();
        if(led_connecting) led_connecting->turn_off();
        if(led_accelerating) led_accelerating->turn_off();
        if(led_braking) led_braking->turn_off();
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