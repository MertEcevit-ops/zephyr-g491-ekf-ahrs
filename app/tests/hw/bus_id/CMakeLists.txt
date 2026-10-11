cmake_minimum_required(VERSION 3.20.0)

# Overlay, binding and driver come from the application; nothing is copied.
get_filename_component(PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)
set(DTC_OVERLAY_FILE ${PROJECT_ROOT}/boards/nucleo_g491re.overlay)
list(APPEND ZEPHYR_EXTRA_MODULES ${PROJECT_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hw_bus_id LANGUAGES C)

target_sources(app PRIVATE src/test_raw_bus.c)
if(CONFIG_ISM330DHCX)
  target_sources(app PRIVATE
	src/test_drivers.c
	${PROJECT_ROOT}/src/sensor_hub/ism330_spi_only.c
  )
endif()
target_include_directories(app PRIVATE ${PROJECT_ROOT}/drivers/sensor/mmc5983ma)
target_compile_options(app PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
