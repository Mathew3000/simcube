# The application layer, built for the host.
#
# This target is the POINT of the extraction rather than a convenience. If App only built under
# PlatformIO then the Arduino coupling would still be there and nobody would find out until
# somebody tried the port. partsim_console links this and runs the same golden sequence the
# firmware's `g` command runs, on the host, through the same code.
add_library(partsim_app STATIC
  src/App.cpp
  src/Console.cpp
  src/Role.cpp
)
target_include_directories(partsim_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(partsim_app PUBLIC partsim_core)
target_compile_options(partsim_app PRIVATE
  -Wall -Wextra -Wdouble-promotion -Wfloat-conversion -ffp-contract=off -fno-math-errno)
