#include "bme690.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bme690 {

static const char *TAG = "bme690";

void BME690Component::setup() {
  ESP_LOGI(TAG, "Initializing BME690");

  bme69x_i2c_init(&dev_, this->i2c_bus_->get_port(), this->address_);

  bme69x_init(&dev_);

  conf_.os_hum = BME69X_OS_2X;
  conf_.os_temp = BME69X_OS_4X;
  conf_.os_pres = BME69X_OS_4X;
  conf_.filter = BME69X_FILTER_SIZE_3;

  bme69x_set_conf(&conf_, &dev_);

  heat_.enable = BME69X_ENABLE;
  heat_.heatr_temp = 300;
  heat_.heatr_dur = 100;

  bme69x_set_heatr_conf(BME69X_FORCED_MODE, &heat_, &dev_);

  bme69x_set_op_mode(BME69X_FORCED_MODE, &dev_);
}

void BME690Component::update() {
  struct bme69x_data data;
  uint8_t n_fields;

  bme69x_set_op_mode(BME69X_FORCED_MODE, &dev_);
  dev_.delay_us(10000, dev_.intf_ptr);

  if (bme69x_get_data(BME69X_FORCED_MODE, &data, &n_fields, &dev_) == BME69X_OK) {
    if (temperature_)
      temperature_->publish_state(data.temperature);
    if (pressure_)
      pressure_->publish_state(data.pressure / 100.0f);
    if (humidity_)
      humidity_->publish_state(data.humidity);
    if (gas_)
      gas_->publish_state(data.gas_resistance);
  }
}

}  // namespace bme690
}  // namespace esphome
