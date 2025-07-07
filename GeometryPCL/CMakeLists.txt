cmake_minimum_required(VERSION 3.10)
project(PCLIncrementalFusion)

# cmake_policy(SET CMP0074 NEW)
# cmake_policy(SET CMP0144 NEW)  # 支持 <PACKAGE>_ROOT (大写)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置 Boost 路径
# set(Boost_ROOT "D:/Usmiles/UsmileRobotCpp/Utils/PCLlib/PCL1.15.0/3rdParty/Boost")
# set(BOOST_ROOT "${Boost_ROOT}")  # 可选冗余
# set(Boost_NO_SYSTEM_PATHS ON)    # 避免系统 Boost 干扰

# 让 Boost 使用静态链接（如果你的是 .lib）
# set(Boost_USE_STATIC_LIBS ON)
# set(Boost_USE_MULTITHREADED ON)

# 显式指定 Boost 组件（PCL 常用：filesystem, system, thread）
# find_package(Boost REQUIRED COMPONENTS filesystem system thread)

# 找 PCL
set(PCL_DIR "D:/Usmiles/UsmileRobotCpp/Utils/PCLib/PCL 1.13.0/cmake")
find_package(PCL REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# link_directories(${Boost_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# include_directories(${Open3D_INCLUDE_DIRS})
# link_directories(${Open3D_LIBRARY_DIRS})

message(STATUS "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_DEFINITIONS = ${PCL_DEFINITIONS}")


add_executable(PCLIncrementalFusion IcpFusion.cpp)

# 设置编译选项
target_compile_options(PCLIncrementalFusion PRIVATE /bigobj)
# target_link_libraries(PCLIncrementalFusion ${PCL_LIBRARIES} Open3D::Open3D)
target_link_libraries(PCLIncrementalFusion ${PCL_LIBRARIES})
# 添加你的头文件目录（可选）
target_include_directories(PCLIncrementalFusion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
