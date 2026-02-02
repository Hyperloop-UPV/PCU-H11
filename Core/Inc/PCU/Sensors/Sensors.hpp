#pragma once
#include "PCU/Data/Data.hpp"

class Sensors{
    public:
    inline static D1_NC bool gd_fault_a{false};
    inline static D1_NC bool gd_fault_b{false};
    inline static D1_NC bool gd_ready_a{false};
    inline static D1_NC bool gd_ready_b{false};

using DigitalInputInstance = ST_LIB::DigitalInputDomain::Instance;
    inline static DigitalInputInstance* gd_fault_a_instance = nullptr;
    inline static DigitalInputInstance* gd_fault_b_instance = nullptr;
    inline static DigitalInputInstance* gd_ready_a_instance = nullptr;
    inline static DigitalInputInstance* gd_ready_b_instance = nullptr;

    Sensors()=default;
    static void init(DigitalInputInstance& gd_fault_a_inst,
                     DigitalInputInstance& gd_fault_b_inst,
                     DigitalInputInstance& gd_ready_a_inst,
                     DigitalInputInstance& gd_ready_b_inst)
    {
        Sensors::gd_fault_a_instance = &gd_fault_a_inst;
        Sensors::gd_fault_b_instance = &gd_fault_b_inst;
        Sensors::gd_ready_a_instance = &gd_ready_a_inst;
        Sensors::gd_ready_b_instance = &gd_ready_b_inst;
    }

    static void read()
    {
        gd_fault_a = gd_fault_a_instance->read();
        gd_fault_b = gd_fault_b_instance->read();
        gd_ready_a = gd_ready_a_instance->read();
        gd_ready_b = gd_ready_b_instance->read();
    }
};