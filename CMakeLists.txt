cmake_minimum_required(VERSION 3.10)  # Specify the minimum version of CMake required

project(ICM20948_Demo)  # Name of the project

# Set the C++ standard to C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# Add compiler warnings
add_compile_options(-Wall)

# Define the source files for the executable
add_executable(ICM20948_Demo main.c ICM20948.c)

# Link against the math library
target_link_libraries(ICM20948_Demo m)
