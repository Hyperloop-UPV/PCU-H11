#include "Examples/ExampleMPU.cpp"
#include "Examples/ExamplesHardFault.cpp"

#include "main.h"
#include "ST-LIB.hpp"
#include "PCU/PCU.hpp"

int main(void) { 
  Hard_fault_check();
  STLIB::start();
  //Pasar todos los pines aqui y inicializar todo
  using myBoard = ST_LIB::Board<Pinout::encoder_timer>;
  myBoard::init();
  auto encoder_timer_wrapper = get_timer_instance(myBoard, Pinout::encoder_timer);
  ST_LIB::Encoder<Pinout::encoder_timer> enc(encoder_timer_wrapper, 1, 1);
  Speetec::init(&enc);

  while (1) {
    STLIB::update();
  }
}
void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
