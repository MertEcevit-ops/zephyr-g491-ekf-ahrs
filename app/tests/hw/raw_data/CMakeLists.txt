cmake_minimum_required(VERSION 3.20.0)

# Overlay, binding, driver and sensor hub come from the application; nothing is copied.
get_filename_component(PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)
set(DTC_OVERLAY_FILE ${PROJECT_ROOT}/boards/nucleo_g491re.overlay)
list(APPEND ZEPHYR_EXTRA_MODULES ${PROJECT_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hw_raw_data LANGUAGES C)

target_sources(app PRIVATE ${PROJECT_ROOT}/src/sensor_hub/ism330_spi_only.c)

if(CONFIG_MMC5983MA_INT_DIAG)
  # The driver is exercised directly; a running sensor hub would race for it.
  target_sources(app PRIVATE src/test_conversion_time.c)
else()
  target_sources(app PRIVATE
	src/hub_fixture.c
	src/test_bridge_offset.c
	${PROJECT_ROOT}/src/sensor_hub/sensor_hub.c
	${PROJECT_ROOT}/src/sensor_hub/sensor_cal.c
  )
  if(NOT CONFIG_RAW_DATA_OFFSET_ONLY)
    target_sources(app PRIVATE src/test_stream.c)
  endif()
endif()

target_include_directories(app PRIVATE ${PROJECT_ROOT}/src)
target_compile_options(app PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
