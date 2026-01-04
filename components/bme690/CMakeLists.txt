idf_component_register(
    SRCS "bme690.cpp"
    INCLUDE_DIRS "."
)

target_sources(
    ${COMPONENT_LIB}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../../lib/BME690_SensorAPI/bme69x.c
    ${CMAKE_CURRENT_LIST_DIR}/../../lib/BME690_SensorAPI/bme69x_i2c_esp_idf.c
    ${CMAKE_CURRENT_LIST_DIR}/../../lib/BME690_SensorAPI/bme69x_i2c_helper.c
)
