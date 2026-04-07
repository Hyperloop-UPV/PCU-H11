#pragma once

#include "PCU/Sensors/IMU/IMU_Antialias_freq.hpp"
#include "PCU/Sensors/IMU/IMU_Registers.hpp"
#include "ST-LIB_LOW/ErrorHandler/ErrorHandler.hpp"
#include <deque>
#include <cmath>

class IMU{
	static inline double  accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp;

	static inline constexpr double PI_ = 3.1415926535;
	static inline double accel_sensitivity = 16384;
	static inline double gyro_sensitivity = 16.4;

	static inline uint8_t txData[1] = { 0 };
	static inline uint8_t rxData[1] = { 0 };

	static inline std::optional<ST_LIB::SPIDomain::SPIWrapper<Pinout::spi_def>> spi_wrapper;
	static inline ST_LIB::DigitalOutputDomain::Instance* spi_cs = nullptr;
	static inline ST_LIB::SPIDomain::Instance* spi_pins= nullptr;
	static inline double accel_offset_x = 0;
	static inline double accel_offset_y = 0;
	static inline double accel_offset_z = 0;
	
	static inline Integrator<IntegratorType::Trapezoidal> velocity_integrator{1.0 / 3000.0 , 1};

public:

	static void init(ST_LIB::DigitalOutputDomain::Instance& spi_cs_get, ST_LIB::SPIDomain::Instance& spi_pins_get)
	{
		spi_cs = &spi_cs_get;
		spi_pins = &spi_pins_get;
		spi_wrapper.emplace(*spi_pins);
		soft_reset();
		Scheduler::set_timeout(15*1000, [](){
			write_acceleration_config(ACCELERATION_2G, ACCELERATION_4KHZ);
			config_accel_antialias(ANTIALIAS_FREQ_1051_HZ);
			turn_on_sensors();
		});
	
	}

	static double read_imu_x_acceleration(){
		 
		return read_accel_x() - accel_offset_x;
	}

	static double get_imu_x_speed(){
		static double velocity = 0;
		static double acceleration = read_imu_x_acceleration();
		acceleration *= 9.8;

		velocity_integrator.input(acceleration);
		velocity_integrator.execute();
		velocity= velocity_integrator.output_value;
		return velocity * 3.6; // m/s to km/h
	}

	static void restart()
	{
		velocity_integrator.reset();
	}
	// Sync IMU integrator speed with an external reference (e.g. Speetec),
	// provided in km/h.
	static void sync_speed_with_reference(double speed_km_h) {
		double velocity_ms = speed_km_h / 3.6;
		velocity_integrator.reset();
		velocity_integrator.integral = velocity_ms;
		velocity_integrator.output_value = velocity_ms;
		velocity_integrator.first_execution = false;
	}
	}

	static void calibrate(size_t TIMES_TO_CREATE_ZERO = 100) {
		double new_offset_x = 0;
		double new_offset_y = 0;
		double new_offset_z = 0;
		for(size_t i = 1; i < TIMES_TO_CREATE_ZERO; i++) {
			double current_x = read_accel_x();
			double current_y = read_accel_y();
			double current_z = read_accel_z();
			new_offset_x = (new_offset_x * (i - 1) + current_x) / i;
			new_offset_y = (new_offset_y * (i - 1) + current_y) / i;
			new_offset_z = (new_offset_z * (i - 1) + current_z) / i;
		}
		accel_offset_x = new_offset_x;
		accel_offset_y = new_offset_y;
		accel_offset_z = new_offset_z;
	}

	static void read_imu_data(){
		accel_x = read_accel_x() - accel_offset_x;
		accel_y = read_accel_y() - accel_offset_y;
		accel_z = read_accel_z() - accel_offset_z;
	}

	static void get_accelerations(double& x, double& y, double& z){
		read_imu_data();
		x = accel_x;
		y = accel_y;
		z = accel_z;
	}

	static void turn_on_sensors(){
	write_sensors_configuration(TEMPERATURE_ON, RC_ALLWAYS_ON, ACCELERATION_LOW_NOISE);
	// HAL_Delay(5);
	}

	static void turn_off_sensors(){
		write_sensors_configuration(TEMPERATURE_OFF, RC_ALLWAYS_ON, ACCELERATION_OFF);
	// HAL_Delay(5);
	}

private:

	static inline void SPI_transmit(const span<uint8_t> data) {
		spi_wrapper->send_DMA(data);
	}

	static inline void SPI_receive(span<uint8_t> buffer) {
		spi_wrapper->receive_DMA(buffer);
	}

	static uint8_t read_register(uint8_t register_address){
	txData[0] = register_address | 0b10000000;
	spi_cs->turn_off(); 
	SPI_transmit(txData);
	rxData[0] = 0;
	SPI_receive(rxData);
	spi_cs->turn_on(); 
	return rxData[0];
	}

	static bool write_register(uint8_t register_address, uint8_t content){
	spi_cs->turn_off(); 
	txData[0] = register_address;
	SPI_transmit(txData);
	txData[0] = content;
	SPI_transmit(txData);
	spi_cs->turn_on(); 
	return read_register(register_address) == content;
	}

	static void soft_reset(){
		DeviceConfigRegister reg;
		reg.value = read_register(DEVICE_CONFIG);
		reg.fields.SOFT_RESET_CONFIG = 1;
		write_register(DEVICE_CONFIG, reg.value);
		// HAL_Delay(5);
	}

	static uint8_t read_id(){
		return read_register(0x75);
	}

	static uint8_t read_pwm_mgmt0_fields(){
		return read_register(PWR_MGMT0);
	}

	static void write_sensors_configuration(	TEMPERATURE_OPERACION_MODES temp_dis,
								 	 RC_OPERATION_MODE idle,
									ACCELERATION_OPERATION_MODES accel_mode	){
	SensorConfigRegister reg;
	reg.value = 0;
	reg.fields.TEMP_DIS		= temp_dis;
	reg.fields.ACCEL_MODE	= accel_mode;
	reg.fields.GYRO_MODE	= 0;
	reg.fields.IDLE 		= idle;
	write_register(PWR_MGMT0, reg.value);
	}
	
	static SensorConfigRegister read_sensor_configuration() {
	SensorConfigRegister reg;
	reg.value = read_register(PWR_MGMT0);
	return reg;
	}

	static AccelerationConfigRegister read_acceleration_config(){
	AccelerationConfigRegister reg;
	reg.value = read_register(ACCEL_CONFIG0);
	return reg;
	}

	static void write_acceleration_config(ACCELERATION_FULL_SCALE sf, ACCELERATION_ODR odr){
	AccelerationConfigRegister reg;
	reg.fields.ACCEL_FS_SEL = sf;
	reg.fields.ACCEL_ODR = odr;
	reg.fields.__RESERVED_2 = 0;
	accel_sensitivity = ACCLERATION_FS_SENSITIVITY[(uint8_t) sf];
	write_register(ACCEL_CONFIG0, reg.value);
	}

	static void config_accel_antialias(uint16_t freq){
		static uint16_t last_freq = 0;
		last_freq = freq;
		turn_off_sensors();
		Scheduler::set_timeout(5*1000, [](){
			AntialiasConfig config = ANTIALIAS_FREQ_TO_FILTER_CONFIG[last_freq];
			AccelConfigStatic2Register config_static2;
			AccelConfigStatic3Register config_static3;
			AccelConfigStatic4Register config_static4;


			config_static2.fields.ACCEL_AAF_DIS  = FILTER_ON;
			config_static2.fields.ACCEL_AAF_DELT = config.AAF_DELT;
			config_static2.fields._RESERVED = 0;

			config_static3.fields.ACCEL_AAF_DELTSQR = config.AAF_DELTSQR & 0b0000000011111111;

			config_static4.fields.ACCEL_AAF_BITSHIFT = config.AAF_BITSHIFT;
			config_static4.fields.ACCEL_AAF_DELTSQR  = (config.AAF_DELTSQR & 0b0000111100000000) >> 8;

			write_register(ACCEL_CONFIG_STATIC2, config_static2.value);
			write_register(ACCEL_CONFIG_STATIC3, config_static3.value);
			write_register(ACCEL_CONFIG_STATIC4, config_static4.value);
			turn_on_sensors();
		});
	}

	static double read_temp(){
		int16_t raw = (read_register(TEMP_DATA1) << 8) | read_register(TEMP_DATA0);
		return (raw / 132.48) + 25;
	}


	static double read_accel_x(){
		int16_t raw = (int16_t) (read_register(ACCEL_DATA_X1) << 8) | read_register(ACCEL_DATA_X0);
		return  raw / accel_sensitivity;
	}

	static double read_accel_y(){
		int16_t raw = (int16_t) (read_register(ACCEL_DATA_Y1) << 8) | read_register(ACCEL_DATA_Y0);
		return  raw / accel_sensitivity;
	}

	static double read_accel_z(){
		int16_t raw = (int16_t) (read_register(ACCEL_DATA_Z1) << 8) | read_register(ACCEL_DATA_Z0);
		return  raw / accel_sensitivity;
	}

};

