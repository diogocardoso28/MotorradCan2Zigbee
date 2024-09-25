idf_component_register(SRCS "temp_sensor_driver.c" "switch_driver.c" "motorrad_can_2_zigbee.c" 
                    INCLUDE_DIRS "." "../managed_components/espressif__esp-zigbee-lib/include/zcl" )