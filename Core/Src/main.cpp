#include "main.h"

#include "ST-LIB.hpp"
#include "main.h"
#include "PCU/PCU.hpp"

using ST_LIB::EthernetDomain;
TIM_TypeDef* global_us_timer = nullptr;

void pcu_control_callback(void* raw) {
    if(PCU::control_data.space_vector_active == SpaceVectorState::ACTIVE && PCU::callback_flag)
    {
        SpaceVector::calculate_duties();

        if(CurrentControl::is_running())
        {
            CurrentControl::control_action();
        }
    }
    
}

#define PCU_MAC_ADDR "02:11:22:33:44:55"
#define PCU_IP_ADDR  "192.168.1.5"

#if defined(USE_PHY_LAN8742)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, PCU_MAC_ADDR,
                             PCU_IP_ADDR, "255.255.0.0");
#elif defined(USE_PHY_LAN8700)
#if MASK_TO_24 == 1
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, PCU_MAC_ADDR,
                             PCU_IP_ADDR, "255.255.255.0");
#else
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, PCU_MAC_ADDR,
                             PCU_IP_ADDR, "255.255.0.0");
#endif
#elif defined(USE_PHY_KSZ8041)
#if MASK_TO_24 == 1
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H11, PCU_MAC_ADDR,
                             PCU_IP_ADDR, "255.255.255.0");
#else
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H11, PCU_MAC_ADDR,
                             PCU_IP_ADDR, "255.255.0.0");
#endif
#else
#error "Ethernet PHY not defined"

#endif

ST_LIB::DigitalOutputDomain::Instance *led_connecting = 0;
ST_LIB::DigitalOutputDomain::Instance *led_operational = 0;
ST_LIB::DigitalOutputDomain::Instance *led_fault = 0;

bool initialized_stlib = false;

int main(void) {

  Hard_fault_check();
  #if PCU_H10 == 1
  using myBoard = ST_LIB::Board<PCUFaultPolicy, eth,Pinout::tim_encoder_decl,Pinout::tim_decl, Pinout::Buff_enable, Pinout::Reset_bypass,
                               Pinout::led_connecting, Pinout::led_fault, Pinout::led_operational,
                               Pinout::FAULT_GD_INVERTER_A,Pinout::FAULT_GD_INVERTER_B,
                               Pinout::READY_GD_INVERTER_A,Pinout::READY_GD_INVERTER_B,
                               Pinout::Voltage_Battery_A,Pinout::Voltage_Battery_B,
                               Pinout::Current_sensor_U_A, Pinout::Current_sensor_U_B,
                               Pinout::Current_sensor_V_A, Pinout::Current_sensor_V_B,
                               Pinout::Current_sensor_W_A, Pinout::Current_sensor_W_B,Pinout::general_purpose_timer,
                               Pinout::spi_cs_def,Pinout::spi_def,
                               PCU_Protections::voltage_A, PCU_Protections::voltage_B,
                               PCU_Protections::current_u_a, PCU_Protections::current_v_a, PCU_Protections::current_w_a,
                               PCU_Protections::current_u_b, PCU_Protections::current_v_b, PCU_Protections::current_w_b, PCU_Protections::max_theoretical_speed,
#ifdef TEST_LEDS
                               led_flash_def, led_sleep_def, led_LIM1_def, led_LIM2_def,
#endif
                               PCU_Protections::position, PCU_Protections::space_vector_time>;

  #else
  using myBoard = ST_LIB::Board<PCUFaultPolicy, eth,Pinout::tim_encoder_decl,Pinout::tim_decl, Pinout::Buff_enable, Pinout::Reset_bypass,
                               Pinout::led_connecting, Pinout::led_fault, Pinout::led_operational, Pinout::led_accelerating, Pinout::led_braking,
                               Pinout::FAULT_GD_INVERTER_A,Pinout::FAULT_GD_INVERTER_B,
                               Pinout::READY_GD_INVERTER_A,Pinout::READY_GD_INVERTER_B,
                               Pinout::Voltage_Battery_A,Pinout::Voltage_Battery_B,
                               Pinout::Current_sensor_U_A, Pinout::Current_sensor_U_B,
                               Pinout::Current_sensor_V_A, Pinout::Current_sensor_V_B,
                               Pinout::Current_sensor_W_A, Pinout::Current_sensor_W_B,Pinout::general_purpose_timer,
                               Pinout::Speetec_supply, Pinout::Hall_SupplyA, Pinout::Hall_SupplyB,
                               PCU_Protections::voltage_A, PCU_Protections::voltage_B,
                               PCU_Protections::current_u_a, PCU_Protections::current_v_a, PCU_Protections::current_w_a,
                               PCU_Protections::current_u_b, PCU_Protections::current_v_b, PCU_Protections::current_w_b, PCU_Protections::max_theoretical_speed,
                               PCU_Protections::position, PCU_Protections::space_vector_time>;
  #endif

  myBoard::init();
  initialized_stlib = true;
  init_adj_commit_hash_check();

  #if PCU_H10 == 1
  led_connecting = &myBoard::instance_of<Pinout::led_connecting>();
  led_fault = &myBoard::instance_of<Pinout::led_fault>();
  led_operational = &myBoard::instance_of<Pinout::led_operational>();

  auto& fault_inverter_a = myBoard::instance_of<Pinout::FAULT_GD_INVERTER_A>();
  auto& fault_inverter_b = myBoard::instance_of<Pinout::FAULT_GD_INVERTER_B>();
  auto& ready_inverter_a = myBoard::instance_of<Pinout::READY_GD_INVERTER_A>();
  auto& ready_inverter_b = myBoard::instance_of<Pinout::READY_GD_INVERTER_B>();

  auto& reset_bypass = myBoard::instance_of<Pinout::Reset_bypass>();
  auto& buff_enable = myBoard::instance_of<Pinout::Buff_enable>();

  auto& Voltage_A = myBoard::instance_of<Pinout::Voltage_Battery_A>();
  auto& Voltage_B = myBoard::instance_of<Pinout::Voltage_Battery_B>();

  auto& current_sensor_u_a = myBoard::instance_of<Pinout::Current_sensor_U_A>();
  auto& current_sensor_u_b = myBoard::instance_of<Pinout::Current_sensor_U_B>();
  auto& current_sensor_v_a = myBoard::instance_of<Pinout::Current_sensor_V_A>();
  auto& current_sensor_v_b = myBoard::instance_of<Pinout::Current_sensor_V_B>();
  auto& current_sensor_w_a = myBoard::instance_of<Pinout::Current_sensor_W_A>();
  auto& current_sensor_w_b = myBoard::instance_of<Pinout::Current_sensor_W_B>();

  auto& spi_pins = myBoard::instance_of<Pinout::spi_def>();
  auto& spi_cs = myBoard::instance_of<Pinout::spi_cs_def>();
  IMU::init(spi_cs, spi_pins);

  Actuators::init(buff_enable, reset_bypass,
                  *led_connecting, *led_fault, *led_operational);

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

  auto& Voltage_A = myBoard::instance_of<Pinout::Voltage_Battery_A>();
  auto& Voltage_B = myBoard::instance_of<Pinout::Voltage_Battery_B>();

  // auto& ppu_temp_a = myBoard::instance_of<Pinout::PPU_temp_A>();
  // auto& ppu_temp_b = myBoard::instance_of<Pinout::PPU_temp_B>();

  auto& current_sensor_u_a = myBoard::instance_of<Pinout::Current_sensor_U_A>();
  auto& current_sensor_u_b = myBoard::instance_of<Pinout::Current_sensor_U_B>();
  auto& current_sensor_v_a = myBoard::instance_of<Pinout::Current_sensor_V_A>();
  auto& current_sensor_v_b = myBoard::instance_of<Pinout::Current_sensor_V_B>();
  auto& current_sensor_w_a = myBoard::instance_of<Pinout::Current_sensor_W_A>();
  auto& current_sensor_w_b = myBoard::instance_of<Pinout::Current_sensor_W_B>();

  Actuators::init(hall_supply_a, hall_supply_b,speedtec_supply, buff_enable, reset_bypass);
  Actuators::init_leds(led_operational, led_fault, led_connecting,
                       led_accelerating, led_braking);

    
  #endif
    
  auto timer = get_timer_instance(myBoard, Pinout::tim_decl);
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::U_PWM_pin,Pinout::U_PWM_negated_pin> pwm_u= timer.get_dual_pwm<Pinout::U_PWM_pin, Pinout::U_PWM_negated_pin>();
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::V_PWM_pin,Pinout::V_PWM_negated_pin> pwm_v= timer.get_dual_pwm<Pinout::V_PWM_pin, Pinout::V_PWM_negated_pin>();
  ST_LIB::DualPWM<Pinout::tim_decl,Pinout::W_PWM_pin,Pinout::W_PWM_negated_pin> pwm_w= timer.get_dual_pwm<Pinout::W_PWM_pin, Pinout::W_PWM_negated_pin>();
  PWMActuators::init(pwm_u, pwm_v, pwm_w);

  auto timer2 = get_timer_instance(myBoard, Pinout::tim_encoder_decl);
  
  ST_LIB::Encoder<Pinout::tim_encoder_decl> encoder = 
        timer2.get_encoder();

  Speetec::init(&encoder);

  CurrentSensors::init(current_sensor_u_a, current_sensor_u_b,
                       current_sensor_v_a, current_sensor_v_b,
                       current_sensor_w_a, current_sensor_w_b);
  VoltageSensors::init(Voltage_A, Voltage_B);

  
  Sensors::init(fault_inverter_a, fault_inverter_b,
                ready_inverter_a, ready_inverter_b);

  auto tim_gp0 = get_timer_instance(myBoard, Pinout::general_purpose_timer);
  tim_gp0.set_prescaler(tim_gp0.get_clock_frequency() / 1000'000);
  tim_gp0.configure16bit(pcu_control_callback, nullptr, 199);
  tim_gp0.enable_nvic();
  tim_gp0.enable_update_interrupt();

  ethernet = &myBoard::instance_of<eth>();
  PCU::start();

  for(uint32_t i = 0; i < 20000; i++) {
    Scheduler::update();
  }

#if 0
  {
    zeroing();
    IMU::restart();
    speed_integrator.reset();
    #if PCU_H10 == 1
    IMU::calibrate(10000);
    #endif
  }

#endif

  Comms::start();

  Watchdog::watchdog_time = std::chrono::milliseconds(500);
  Watchdog::start();

  while (1) {
    Watchdog::refresh();
    Scheduler::update();
    PCU::update();
    ethernet->update();
  }
}

extern "C" void Error_Handler(void) {
    PANIC("HAL error handler triggered");
    while (1) {
    }
}
