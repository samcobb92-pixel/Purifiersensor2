import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_TEMPERATURE,
    CONF_PRESSURE,
    CONF_HUMIDITY,
    CONF_GAS_RESISTANCE,
)

DEPENDENCIES = ["i2c"]

bme690_ns = cg.esphome_ns.namespace("bme690")
BME690Sensor = bme690_ns.class_("BME690Component", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement="°C", accuracy_decimals=2
            ),
            cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
                unit_of_measurement="hPa", accuracy_decimals=1
            ),
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement="%", accuracy_decimals=1
            ),
            cv.Optional(CONF_GAS_RESISTANCE): sensor.sensor_schema(
                unit_of_measurement="Ω", accuracy_decimals=0
            ),
        }
    )
    .extend(i2c.i2c_device_schema(0x76))
    .extend(cv.polling_component_schema("60s"))
)

async def to_code(config):
    var = cg.new_Pvariable(config["id"])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_PRESSURE in config:
        sens = await sensor.new_sensor(config[CONF_PRESSURE])
        cg.add(var.set_pressure_sensor(sens))

    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity_sensor(sens))

    if CONF_GAS_RESISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_GAS_RESISTANCE])
        cg.add(var.set_gas_sensor(sens))
