#include "espIna.h"


espIna::espIna(){
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = INA_I2C_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = INA_I2C_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  conf.clk_flags = 0;

  i2c_param_config(INA_I2C_MASTER_PORT, &conf);
  ESP_ERROR_CHECK(i2c_driver_install(INA_I2C_MASTER_PORT, conf.mode, 0, 0, 0));
}

esp_err_t espIna::_read_reg(uint8_t reg_addr, uint16_t * reg_value){
  uint8_t read_buffer[2];
  i2c_master_write_read_device(INA_I2C_MASTER_PORT, INA_I2C_ADDR, &reg_addr, 1, read_buffer, 2, INA_I2C_TIMEOUT / portTICK_PERIOD_MS);
  *reg_value = (read_buffer[1] << 8) + (read_buffer[0]);
  return ESP_OK;
}

esp_err_t espIna::_write_reg(uint8_t reg_addr, uint16_t reg_value){
  uint8_t write_buffer[3];
  write_buffer[0] = reg_addr;
  write_buffer[1] = reg_value >> 8;
  write_buffer[2] = reg_value & 0xFF; 

  ESP_ERROR_CHECK(i2c_master_write_to_device(INA_I2C_MASTER_PORT, INA_I2C_ADDR, write_buffer, sizeof(write_buffer), INA_I2C_TIMEOUT / portTICK_PERIOD_MS));
  return ESP_OK;
}

esp_err_t espIna::set_calibration(float r_shunt_res_ohm, float max_current_a){
  esp_err_t ret_value = ESP_FAIL;
  _current_lsb = max_current_a / (float)pow(2U, 15U);
  _power_lsb = 20 * _current_lsb;
  uint16_t calibration_value = trunc(0.04096F / ( _current_lsb * r_shunt_res_ohm ));

  _write_reg(INA_CALIB_REG_ADDR, calibration_value);

  ret_value = ESP_OK;

  return ret_value;
}

esp_err_t espIna::adjust_current_offset(float meas_current_a) {
  esp_err_t ret_value = ESP_FAIL;
  uint16_t calibration_value = 0U;
  uint16_t adj_calibration_value = 0U;
  uint16_t current_reg_value = 0U;
  
  ESP_ERROR_CHECK(_read_reg(INA_CALIB_REG_ADDR, &calibration_value));
  ESP_ERROR_CHECK(_read_reg(INA_CURRENT_REG_ADDR, &current_reg_value));

  adj_calibration_value = trunc(calibration_value * meas_current_a / current_reg_value);

  ESP_ERROR_CHECK(_write_reg(INA_CALIB_REG_ADDR, adj_calibration_value));

  ret_value = ESP_OK;

  return ret_value;
}

esp_err_t espIna::get_bus_current(float * current_a){
  esp_err_t ret_value = ESP_FAIL;
  uint16_t curr_reg_value = 0U;
  ESP_ERROR_CHECK(_read_reg(INA_CURRENT_REG_ADDR, &curr_reg_value));

  *current_a = (float)((int16_t)curr_reg_value * _current_lsb);

  return ret_value;
}

esp_err_t espIna::get_bus_voltage(float * voltage_v){
  esp_err_t ret_value = ESP_FAIL;
  uint16_t volt_reg_value = 0U;
  float voltage_lsb = 4.F / 1000.F;

  ESP_ERROR_CHECK(_read_reg(INA_BUS_VOLT_REG_ADDR, &volt_reg_value));
  *voltage_v = (float)( (volt_reg_value >> 3)  * voltage_lsb );
  overflow = volt_reg_value & 0x1;
  conversion_ready = (volt_reg_value >> 1) & 0x1;

  return ret_value;
}

esp_err_t espIna::get_bus_power(float * bus_power_w){
  esp_err_t ret_value = ESP_FAIL;
  uint16_t power_reg_value = 0U;

  ESP_ERROR_CHECK(_read_reg(INA_POWER_REG_ADDR, &power_reg_value));
  *bus_power_w = (float)( (int16_t)power_reg_value  * _power_lsb );

  return ret_value;
}

