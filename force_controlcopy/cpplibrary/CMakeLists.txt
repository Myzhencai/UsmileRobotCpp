cmake_minimum_required(VERSION 3.10)
project(RobotUtilities)
set(CMAKE_BUILD_TYPE Release)

add_compile_options(-std=c++17)

include_directories(include/)

# find_package(Eigen3 3.4 REQUIRED)
set(Eigen3_DIR "D:/force_control/thirdparty/eigen-3.4.0")
# include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Eigen3_DIR})

# find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)
include_directories("D:/force_control/thirdparty/yaml-cpp/include")
link_directories("D:/force_control/thirdparty/yaml-cpp/lib")

add_compile_options(/bigobj)


###########
## Build ##
###########
# add_library(Utilities SHARED
# 	src/spatial_utilities.cpp
# 	src/timer_linux.cpp
#     src/butterworth.cpp
# )

add_library(Utilities STATIC
	src/spatial_utilities.cpp
	src/timer_linux.cpp
    src/butterworth.cpp
)

target_compile_options(Utilities PRIVATE /bigobj)
# target_link_libraries(Utilities ${YAMLLib})
target_link_libraries(Utilities  yaml-cpp)

# add_executable(test_utilities src/test.cpp)
# target_link_libraries(test_utilities Utilities)

#############
## Install ##
#############
# headers

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
    )

# binaries
install(TARGETS Utilities
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/
    )