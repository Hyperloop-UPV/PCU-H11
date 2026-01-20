
#include "main.h"
#include "ST-LIB.hpp"
#include "stm32h7xx_hal.h"
#include "core_cm7.h"


void usage_fault_example(void) {
    __builtin_trap();
}
void func(){
    usage_fault_example();
}
void f(){
    func();
}
void a(){
    f();
}
void busfault_example(void){
    *(uint32_t *)0xdead0000 = 0x20;
}
void memmanage_example(void) {
    void (*fn_in_xn_region)(void) = (void (*)(void))0xE0000000;
    fn_in_xn_region(); 
}

int main(void) {
#ifdef SIM_ON
    SharedMemory::start();
#endif
    Hard_fault_check();
    a();
    HAL_Init();
    DigitalOutput led_on(PB0);
    STLIB::start();
    Time::register_low_precision_alarm(100, [&]() { led_on.toggle(); 
    });

    while (1) {
        STLIB::update();
    }
}

void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
