import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TEMPERATURE,
    CONF_PRESSURE,
    CONF_HUMIDITY,
    CONF_GAS_RESISTANCE,
    CONF_PARENT
)

from . import BME690Component

DEPENDENCIES = ["bme690"]

# define the schema
CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_PARENT): cv.use_id(BME690Component),
    cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(unit_of_measurement="°C", accuracy_decimals=2),
    cv.Optional(CONF_PRESSURE): sensor.sensor_schema(unit_of_measurement="hPa", accuracy_decimals=1),
    cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(unit_of_measurement="%", accuracy_decimals=1),
    cv.Optional(CONF_GAS_RESISTANCE): sensor.sensor_schema(unit_of_measurement="Ω", accuracy_decimals=0),
}).extend(sensor.PLATFORM_SCHEMA)  # must extend PLATFORM_SCHEMA

async def to_code(config):
    parent = await cg.get_variable(config[CONF_PARENT])

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(parent.set_temperature_sensor(sens))

    if CONF_PRESSURE in config:
        sens = await sensor.new_sensor(config[CONF_PRESSURE])
        cg.add(parent.set_pressure_sensor(sens))

    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(parent.set_humidity_sensor(sens))

    if CONF_GAS_RESISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_GAS_RESISTANCE])
        cg.add(parent.set_gas_sensor(sens))
