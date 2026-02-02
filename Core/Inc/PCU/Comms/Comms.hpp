#include "Communications/Packets/OrderPackets.hpp"
#include "Communications/Packets/DataPackets.hpp"


void Start_SVPWM_cb()
{
    Comms::on_start_svpwm = true;
}
void Stop_Motor_cb()
{
    Comms::on_stop_motor = true;
}
void Send_Reference_Current_cb()
{
    Comms::on_send_reference_current =  true;
}
void Zeroing_cb()
{
    Comms::on_zeroing = true;
}
void Send_Reference_Speed_cb()
{
    Comms::on_send_reference_speed = true;
}
void Start_Precharge_cb()
{
    Comms::on_start_precharge = true;
}

class Comms
{
    public:
    inline static bool on_start_svpwm{false};
    inline static bool on_stop_motor{false};
    inline static bool on_send_reference_current{false};
    inline static bool on_zeroing{false};
    inline static bool on_send_reference_speed{false};
    inline static bool on_start_precharge{false};

    inline static OrderPackets order_packets{};
    inline static DataPackets data_packets{};

};