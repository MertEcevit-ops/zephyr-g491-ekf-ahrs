zephyr_library_named(mmc5983ma)
zephyr_library_sources(mmc5983ma.c)
target_compile_options(mmc5983ma PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
