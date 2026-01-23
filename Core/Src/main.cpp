<<<<<<< HEAD
=======
#define EXAMPLE_BASE
#define TEST_0 // Test to be run

// Include all examples, run the one defined above
#include "Examples/ExampleMPU.cpp"

#ifdef EXAMPLE_BASE

>>>>>>> 215797d4569e0740eaf3c91a5831b8cae12c3c55
#include "main.h"
#include "ST-LIB.hpp"

int main(void) {
#ifdef SIM_ON
    SharedMemory::start();
#endif

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
<<<<<<< HEAD
=======

#endif
>>>>>>> 215797d4569e0740eaf3c91a5831b8cae12c3c55
