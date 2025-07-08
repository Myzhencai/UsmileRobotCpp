cmake_minimum_required(VERSION 3.13)
project(ArmController)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 路径设置依赖库的路径
set(BOOST_ROOT "D:/DrJ/boost_1_88_0")
set(NLOHMANN_ROOT "../Utils/nlohmann")
set(CodroidApi_ROOT "../Utils/CodroidApi/include")
set(Boost_NO_SYSTEM_PATHS ON)

# 查找 Boost 库
find_package(Boost  REQUIRED COMPONENTS system thread)

# 输出 Boost 信息
message("Boost version: ${Boost_VERSION_STRING}")
message("Boost include dir: ${Boost_INCLUDE_DIRS}")
message("Boost library dir: ${Boost_LIBRARY_DIRS}")

include_directories("D:/Usmiles/YamlRobot/yaml-cpp/include")
link_directories("D:/Usmiles/YamlRobot/yaml-cpp/lib")


set(Open3D_DIR "D:/Usmiles/UsmileRobotCpp/Utils/open3Dlibnew/CMake")
# # 查找 Open3D
find_package(Open3D REQUIRED)

include_directories(${Open3D_INCLUDE_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})


# 生成库文件
# add_library(ArmController SHARED
#     ArmController.cpp
# )

add_library(ArmController STATIC
    ArmController.cpp
)
# End of Selection

# add_executable(ArmController 
#     ArmController.cpp)

# 添加依赖的头文件
target_include_directories(ArmController PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${NLOHMANN_ROOT}
    ${CodroidApi_ROOT}
    ${Open3D_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

message("CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

# 设置编译选项
target_compile_options(ArmController PRIVATE /bigobj)

# 链接 Boost 库
target_link_libraries(ArmController PRIVATE ${Boost_LIBRARIES}
    yaml-cpp
    Open3D::Open3D
)

# 设置导出宏
target_compile_definitions(ArmController PRIVATE
    $<$<CONFIG:Debug>:ARM_CONTROLLER_DEBUG>
    $<$<CONFIG:Release>:ARM_CONTROLLER_RELEASE>
) 