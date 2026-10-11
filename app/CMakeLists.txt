cmake_minimum_required(VERSION 3.20.0)

# The MMC5983MA driver is a Zephyr module (zephyr/module.yml) so that it links
# like an in-tree driver. A zephyr_library() created after find_package(Zephyr)
# is never linked and its device object would silently vanish.
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(imu_ekf LANGUAGES C)

target_sources(app PRIVATE
	src/main.c
	src/sensor_hub/sensor_hub.c
	src/sensor_hub/sensor_cal.c
	src/sensor_hub/ism330_spi_only.c
	src/ekf/ekf_core.c
	src/ekf/ekf_math.c
	src/fusion/fusion.c
	src/telemetry/telemetry.c
)
target_include_directories(app PRIVATE src)
# -Wvla is left out on purpose: Zephyr's LOG_* macros expand to VLAs.
target_compile_options(app PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
