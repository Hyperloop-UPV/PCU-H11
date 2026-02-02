#include "PCU/Actuators/Actuators.hpp"
#include "PCU/PCU.hpp"


#if PCU_H10 == 0
void Actuators::init(DigitalOutputInstance& hall_supply_a_instance,
                     DigitalOutputInstance& hall_supply_b_instance,
                     DigitalOutputInstance& speedtec_supply_instance,
                     DigitalOutputInstance& enable_pin_instance,
                     DigitalOutputInstance& reset_pin_instance)
{
    hall_supply_a = &hall_supply_a_instance;
    hall_supply_b = &hall_supply_b_instance;
    speedtec_supply = &speedtec_supply_instance;
    enable_pin = NegatedPin{enable_pin_instance};
    reset_bypass = &reset_pin_instance;
    enable_pin.turn_off();  
    reset_bypass->turn_off();
    PCU::control_data.buffer_state = BUFFER_STATE::DISABLED;
}
#else
void Actuators::init(DigitalOutputInstance& enable_pin_instance,
                     DigitalOutputInstance& reset_pin_instance,
                     DigitalOutputInstance& led_operational_instance,
                     DigitalOutputInstance& led_fault_instance,
                     DigitalOutputInstance& led_connecting_instance)
    {
        enable_pin = NegatedPin{enable_pin_instance};
        reset_bypass = &reset_pin_instance;
        led_operational = &led_operational_instance;
        led_fault = &led_fault_instance;
        led_connecting = &led_connecting_instance;

        enable_pin.turn_off();  
        reset_bypass->turn_on();
        led_operational->turn_off();
        led_fault->turn_off();
        led_connecting->turn_off();
        PCU::control_data.buffer_state = BUFFER_STATE::DISABLED;
    }
#endif

void Actuators::enable_buffer()
{
    enable_pin.turn_on();
    PCU::control_data.buffer_state= BUFFER_STATE::ENABLED;
}

void Actuators::disable_buffer()
{
    enable_pin.turn_off();
    PCU::control_data.buffer_state = BUFFER_STATE::DISABLED;
}

void Actuators::enable_reset_bypass()
{
    if(reset_bypass) reset_bypass->turn_on();
}
void Actuators::disable_reset_bypass()
{
    if(reset_bypass) reset_bypass->turn_off();
}

#if PCU_H10 == 0
void Actuators::enable_hall_supply()
{
    if(hall_supply_a) hall_supply_a->turn_on();
    if(hall_supply_b) hall_supply_b->turn_on();
}

void Actuators::disable_hall_supply()
{
    if(hall_supply_a) hall_supply_a->turn_off();
    if(hall_supply_b) hall_supply_b->turn_off();
}

void Actuators::enable_speedtec_supply()
{
    if(speedtec_supply) speedtec_supply->turn_on();
}

void Actuators::disable_speedtec_supply()
{
    if(speedtec_supply) speedtec_supply->turn_off();
}
#endif

/*-----Leds----*/
void Actuators::set_led_operational(bool state)
{
    if(!led_operational) return;
    if(state)
        led_operational->turn_on();
    else
        led_operational->turn_off();
}
void Actuators::set_led_fault(bool state)
{
    if(!led_fault) return;
    if(state)
        led_fault->turn_on();
    else
        led_fault->turn_off();
}
void Actuators::set_led_connecting(bool state)
{
    if(!led_connecting) return;
    if(state)
        led_connecting->turn_on();
    else
        led_connecting->turn_off();
}
#if PCU_H10 == 0
void Actuators::set_led_accelerating(bool state)
{
    if(!led_accelerating) return;
    if(state)
        led_accelerating->turn_on();
    else
        led_accelerating->turn_off();
}
void Actuators::set_led_braking(bool state)
{
    if(!led_braking) return;
    if(state)
        led_braking->turn_on();
    else
        led_braking->turn_off();
}
#endif