cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ekf_tests LANGUAGES C)

set(APP_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

target_sources(app PRIVATE
	src/test_ekf_math.c
	src/test_ekf_core.c
	src/sim.c
	${APP_ROOT}/src/ekf/ekf_math.c
	${APP_ROOT}/src/ekf/ekf_core.c
)
target_include_directories(app PRIVATE ${APP_ROOT}/src/ekf)
target_compile_options(app PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
