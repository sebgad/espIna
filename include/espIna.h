#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"

// Reset bit for INA during power on
#define INA_RESET_BIT 0b1 << 15

// Define bus voltage settings for INA
#define INA_BUS_VOLT_16V 0b0 << 13
#define INA_BUS_VOLT_32V 0b1 << 13

// PGA settings
#define INA_PGA_40_MV 0b00 << 12
#define INA_PGA_80_MV 0b01 << 12
#define INA_PGA_160_MV 0b10 << 12
#define INA_PGA_320_MV 0b11 << 12

// Resolution or number of samples
#define INA_ADC_BUS_RES_9_BIT 0b0000 << 10
#define INA_ADC_BUS_RES_10_BIT 0b0001 << 10
#define INA_ADC_BUS_RES_11_BIT 0b0010 << 10
#define INA_ADC_BUS_RES_12_BIT 0b0011 << 10
#define INA_ADC_BUS_SAM_2_PER_S 0b1001 << 10
#define INA_ADC_BUS_SAM_4_PER_S 0b1010 << 10
#define INA_ADC_BUS_SAM_8_PER_S 0b1011 << 10
#define INA_ADC_BUS_SAM_16_PER_S 0b1100 << 10
#define INA_ADC_BUS_SAM_32_PER_S 0b1101 << 10
#define INA_ADC_BUS_SAM_64_PER_S 0b1110 << 10
#define INA_ADC_BUS_SAM_128_PER_S 0b1111 << 10

#define INA_ADC_SHUNT_RES_9_BIT 0b0000 << 6
#define INA_ADC_SHUNT_RES_10_BIT 0b0001 << 6
#define INA_ADC_SHUNT_RES_11_BIT 0b0010 << 6
#define INA_ADC_SHUNT_RES_12_BIT 0b0011 << 6
#define INA_ADC_SHUNT_SAM_2_PER_S 0b1001 << 6
#define INA_ADC_SHUNT_SAM_4_PER_S 0b1010 << 6
#define INA_ADC_SHUNT_SAM_8_PER_S 0b1011 << 6
#define INA_ADC_SHUNT_SAM_16_PER_S 0b1100 << 6
#define INA_ADC_SHUNT_SAM_32_PER_S 0b1101 << 6
#define INA_ADC_SHUNT_SAM_64_PER_S 0b1110 << 6
#define INA_ADC_SHUNT_SAM_128_PER_S 0b1111 << 6

// Define mode settings for INA
#define INA_MODE_POWER_DOWN 0b000
#define INA_MODE_SHUNT_VOLT_TRIGGERED 0b001
#define INA_MODE_BUS_VOLT_TRIGGERED 0b010
#define INA_MODE_SHUNT_BUS_VOLT_TRIGGERED 0b011
#define INA_MODE_ADC_OFF 0b100
#define INA_MODE_BUS_VOLT_CONTINOUS 0b110
#define INA_MODE_SHUNT_BUS_CONTINOUS 0b111

#define INA_CONFIG_REG_DEFAULT 0x399F

// Register addresses
#define INA_CONFIG_REG_ADDR 0x0
#define INA_SHUNT_VOLT_REG_ADDR 0x1
#define INA_BUS_VOLT_REG_ADDR 0x2
#define INA_POWER_REG_ADDR 0x3
#define INA_CURRENT_REG_ADDR 0x4
#define INA_CALIB_REG_ADDR 0x5

#define INA_I2C_SCL_IO 22
#define INA_I2C_SDA_IO 21
#define INA_I2C_TIMEOUT 1000
#define INA_I2C_ADDR 0b1000000
#define INA_I2C_MASTER_PORT 0

class espIna
{
    public:
        espIna();
        void begin();
		esp_err_t set_bus_voltage();
		esp_err_t set_pga_level();
		esp_err_t set_resolution();
		esp_err_t set_mode();
        esp_err_t set_calibration(float r_shunt_res, float max_current_ma);
        esp_err_t adjust_current_offset(float meas_current);

        esp_err_t get_bus_voltage(float * bus_voltage_v);
        esp_err_t get_bus_current(float * bus_current_a);
        esp_err_t get_bus_power(float * bus_power_w);

        bool overflow = false;
        bool conversion_ready = false;

    private:
        esp_err_t _write_reg(uint8_t reg_addr, uint16_t reg_value);
        esp_err_t _read_reg(uint8_t reg_addr, uint16_t * reg_value);
        int _shunt_resistance;
        float _current_lsb;
        float _power_lsb;
};