#pragma once 
#include "PCU/Data/Data.hpp"
#include "PCU/Control/SpeedControl.hpp"

class RUNS
{
    private:
        inline static float vel_ref{0.0};

    public:
        static void init();
        static float update();
        static void start(uint8_t run_id);
};