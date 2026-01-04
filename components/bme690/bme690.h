#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

extern "C" {
#include "bme69x.h"
#include "bme69x_i2c_esp_idf.h"
}

namespace esphome {
namespace bme690 {

class BME690Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;

  void set_temperature_sensor(sensor::Sensor *s) { temperature_ = s; }
  void set_pressure_sensor(sensor::Sensor *s) { pressure_ = s; }
  void set_humidity_sensor(sensor::Sensor *s) { humidity_ = s; }
  void set_gas_sensor(sensor::Sensor *s) { gas_ = s; }

 protected:
  struct bme69x_dev dev_{};
  struct bme69x_conf conf_{};
  struct bme69x_heatr_conf heat_{};

  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *pressure_{nullptr};
  sensor::Sensor *humidity_{nullptr};
  sensor::Sensor *gas_{nullptr};
};

}  // namespace bme690
}  // namespace esphome
