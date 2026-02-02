#pragma once
#include "ST-LIB.hpp"
#include "PCU/Data/Data.hpp"
#include "PCU/Sensors/CurrentSensor.hpp"
#include "PCU/Sensors/VoltageSensor.hpp"
#include "PCU/Sensors/Speetec.hpp"
#include "PCU/Actuators/Actuators.hpp"
#include "PCU/Actuators/PWMActuators.hpp"
#include "PCU/Control/SpaceVector.hpp"
#include "PCU/Control/CurrentControl.hpp"
#include "PCU/Control/SpeedControl.hpp"
#include "PCU/Comms/Comms.hpp"
class PCU
{
    public:
    D1_NC inline static Control_Data control_data{};
    inline static bool space_vector_on{false};
    inline static bool speed_control_on{false};
    inline static bool current_control_on{false};

    static constexpr auto state= make_state();
    
};