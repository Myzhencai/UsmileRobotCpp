cmake_minimum_required(VERSION 3.10)
project(PCLIncrementalFusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找 PCL
set(PCL_DIR "D:/Usmiles/UsmileRobotCpp/Utils/PCLib/PCL 1.13.0/cmake")
find_package(PCL REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


message(STATUS "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_DEFINITIONS = ${PCL_DEFINITIONS}")


# add_executable(PCLIncrementalFusion IcpFusion.cpp)

add_library(PCLIncrementalFusion STATIC
IcpFusion.cpp
)

# 设置编译选项
target_compile_options(PCLIncrementalFusion PRIVATE /bigobj)
# target_link_libraries(PCLIncrementalFusion ${PCL_LIBRARIES} Open3D::Open3D)
target_link_libraries(PCLIncrementalFusion ${PCL_LIBRARIES})
# 添加你的头文件目录（可选）
target_include_directories(PCLIncrementalFusion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
