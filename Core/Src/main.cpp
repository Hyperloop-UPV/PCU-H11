#include "Examples/ExampleMPU.cpp"
#include "Examples/ExamplesHardFault.cpp"

#include "main.h"
#include "ST-LIB.hpp"
#include "PCU/PCU.hpp"

int main(void) { 
  Hard_fault_check();
  STLIB::start();
  //Pasar todos los pines aqui y inicializar todo
  // using myBoard = ST_LIB::Board<Pinout::encoder_timer>;
  using myBoard = ST_LIB::Board<Pinout::tim_decl>;
  myBoard::init();
  auto timer = get_timer_instance(myBoard, Pinout::tim_decl);
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::U_PWM_pin,Pinout::U_PWM_negated_pin> pwm_u= timer.get_dual_pwm<Pinout::U_PWM_pin, Pinout::U_PWM_negated_pin>();
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::V_PWM_pin,Pinout::V_PWM_negated_pin> pwm_v= timer.get_dual_pwm<Pinout::V_PWM_pin, Pinout::V_PWM_negated_pin>();
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::W_PWM_pin,Pinout::W_PWM_negated_pin> pwm_w= timer.get_dual_pwm<Pinout::W_PWM_pin, Pinout::W_PWM_negated_pin>();
  PWMActuators::init(pwm_u, pwm_v, pwm_w);
  // auto encoder_timer_wrapper = get_timer_instance(myBoard, Pinout::encoder_timer);
  // ST_LIB::Encoder<Pinout::encoder_timer> enc(encoder_timer_wrapper, 1, 1);
  // Speetec::init(&enc);

  while (1) {
    STLIB::update();
  }
}
void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
