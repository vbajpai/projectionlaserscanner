# ---------------------------------------------------------------------------#
# ------------------------------- CMake -------------------------------------#
# ---------------------------------------------------------------------------#

# minimum supported cmake version
cmake_minimum_required(VERSION 2.6)

# identify the project and programming language
project(plscanner CXX)

# compiler flags passed as arguments 
add_definitions(-Wall -Werror)

# specify the header files location
include_directories(${CMAKE_SOURCE_DIR}/include)

# set output directories for libraries and executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# find external libraries
find_package(OpenCV REQUIRED)

# specify the TARGET executable
file(GLOB plscanner_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cc)
add_executable(plscanner ${plscanner_SOURCES})

# link to external libraries
target_link_libraries(plscanner ${OpenCV_LIBS})

# ---------------------------------------------------------------------------#