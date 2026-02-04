#include "Examples/ExampleMPU.cpp"
#include "Examples/ExamplesHardFault.cpp"

#include "main.h"
#include "ST-LIB.hpp"
#include "PCU/PCU.hpp"

int main(void) { 
  Hard_fault_check();
  #if PCU_H10 == 1
  using myBoard = ST_LIB::Board<Pinout::tim_decl, Pinout::Buff_enable, Pinout::Reset_bypass,
                               Pinout::led_connecting, Pinout::led_fault, Pinout::led_operational,
                               Pinout::FAULT_GD_INVERTER_A,Pinout::FAULT_GD_INVERTER_B,
                               Pinout::READY_GD_INVERTER_A,Pinout::READY_GD_INVERTER_B>;
  
  #else
  using myBoard = ST_LIB::Board<Pinout::tim_decl, Pinout::Buff_enable, Pinout::Reset_bypass,
                               Pinout::led_connecting, Pinout::led_fault, Pinout::led_operational, Pinout::led_accelerating, Pinout::led_braking,
                               Pinout::FAULT_GD_INVERTER_A,Pinout::FAULT_GD_INVERTER_B,
                               Pinout::READY_GD_INVERTER_A,Pinout::READY_GD_INVERTER_B,
                               Pinout::Speetec_supply, Pinout::Hall_SupplyA, Pinout::Hall_SupplyB>;
  #endif
  myBoard::init();

  #if PCU_H10 == 1
  auto& led_connecting = myBoard::instance_of<Pinout::led_connecting>();
  auto& led_fault = myBoard::instance_of<Pinout::led_fault>();
  auto& led_operational = myBoard::instance_of<Pinout::led_operational>();

  auto& fault_inverter_a = myBoard::instance_of<Pinout::FAULT_GD_INVERTER_A>();
  auto& fault_inverter_b = myBoard::instance_of<Pinout::FAULT_GD_INVERTER_B>();
  auto& ready_inverter_a = myBoard::instance_of<Pinout::READY_GD_INVERTER_A>();
  auto& ready_inverter_b = myBoard::instance_of<Pinout::READY_GD_INVERTER_B>();

  auto& reset_bypass = myBoard::instance_of<Pinout::Reset_bypass>();
  auto& buff_enable = myBoard::instance_of<Pinout::Buff_enable>();

  Actuators::init(buff_enable, reset_bypass,
                  led_connecting, led_fault, led_operational);

  #else
  auto& led_connecting = myBoard::instance_of<Pinout::led_connecting>();
  auto& led_fault = myBoard::instance_of<Pinout::led_fault>();
  auto& led_operational = myBoard::instance_of<Pinout::led_operational>();
  auto& led_accelerating = myBoard::instance_of<Pinout::led_accelerating>();
  auto& led_braking = myBoard::instance_of<Pinout::led_braking>();

  auto& fault_inverter_a = myBoard::instance_of<Pinout::FAULT_GD_INVERTER_A>();
  auto& fault_inverter_b = myBoard::instance_of<Pinout::FAULT_GD_INVERTER_B>();
  auto& ready_inverter_a = myBoard::instance_of<Pinout::READY_GD_INVERTER_A>();
  auto& ready_inverter_b = myBoard::instance_of<Pinout::READY_GD_INVERTER_B>();

  auto& reset_bypass = myBoard::instance_of<Pinout::Reset_bypass>();
  auto& buff_enable = myBoard::instance_of<Pinout::Buff_enable>();
  auto& speedtec_supply = myBoard::instance_of<Pinout::Speetec_supply>();
  auto& hall_supply_a = myBoard::instance_of<Pinout::Hall_SupplyA>();
  auto& hall_supply_b = myBoard::instance_of<Pinout::Hall_SupplyB>();

  Actuators::init(hall_supply_a, hall_supply_b,speedtec_supply, buff_enable, reset_bypass);
  Actuators::init_leds(led_operational, led_fault, led_connecting,
                       led_accelerating, led_braking);
  #endif
    
  auto timer = get_timer_instance(myBoard, Pinout::tim_decl);
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::U_PWM_pin,Pinout::U_PWM_negated_pin> pwm_u= timer.get_dual_pwm<Pinout::U_PWM_pin, Pinout::U_PWM_negated_pin>();
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::V_PWM_pin,Pinout::V_PWM_negated_pin> pwm_v= timer.get_dual_pwm<Pinout::V_PWM_pin, Pinout::V_PWM_negated_pin>();
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::W_PWM_pin,Pinout::W_PWM_negated_pin> pwm_w= timer.get_dual_pwm<Pinout::W_PWM_pin, Pinout::W_PWM_negated_pin>();
  PWMActuators::init(pwm_u, pwm_v, pwm_w);
  
  Sensors::init(fault_inverter_a, fault_inverter_b,
                ready_inverter_a, ready_inverter_b);

  PCU::start();
  Scheduler::start();

  while (1) {
    PCU::update();
    Scheduler::update();
    STLIB::update();
  }
}
void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
