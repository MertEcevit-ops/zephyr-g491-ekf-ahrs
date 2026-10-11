cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_hub_tests LANGUAGES C)

set(APP_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

target_sources(app PRIVATE
	src/test_sensor_cal.c
	src/test_mmc_decode.c
	${APP_ROOT}/src/sensor_hub/sensor_cal.c
)
target_include_directories(app PRIVATE
	${APP_ROOT}/src
	${APP_ROOT}/src/sensor_hub
	${APP_ROOT}/drivers/sensor/mmc5983ma
)
target_compile_options(app PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
