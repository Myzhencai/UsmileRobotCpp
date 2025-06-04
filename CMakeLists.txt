# cmake_minimum_required(VERSION 3.10)
# project(UsmileRobotCpp)

# # 设置C++标准
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # 设置编译选项
# if(MSVC)
#     add_compile_options(/W4 /WX)
# else()
#     add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# endif()

# # 设置输出目录
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# # 添加所有子目录
# add_subdirectory(SystemStateMachine)
# add_subdirectory(DeviceManager)
# add_subdirectory(ArmController)
# add_subdirectory(CalibrationManager)
# add_subdirectory(Scanner)
# add_subdirectory(TrajectoryPlanner)
# add_subdirectory(ForceMonitor)
# add_subdirectory(CleaningEvaluator)

# # 添加可执行文件
# add_executable(${PROJECT_NAME} main.cpp)

# # 设置包含目录
# target_include_directories(${PROJECT_NAME} PRIVATE
#     ${CMAKE_SOURCE_DIR}
# )

# # 链接所有依赖库
# target_link_libraries(${PROJECT_NAME} PRIVATE
#     SystemStateMachine
#     DeviceManager
#     ArmController
#     CalibrationManager
#     Scanner
#     TrajectoryPlanner
#     ForceMonitor
#     CleaningEvaluator
# ) 


cmake_minimum_required(VERSION 3.10)
project(UsmileRobotCpp)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 添加所有子目录
add_subdirectory(SystemStateMachine)
add_subdirectory(DeviceManager)
add_subdirectory(ArmController)
add_subdirectory(CalibrationManager)
add_subdirectory(Scanner)
add_subdirectory(TrajectoryPlanner)
add_subdirectory(ForceMonitor)
add_subdirectory(CleaningEvaluator)

# 添加可执行文件
add_executable(${PROJECT_NAME} main.cpp)

# 设置包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
)

# 链接所有依赖库
target_link_libraries(${PROJECT_NAME} PRIVATE
    SystemStateMachine
    DeviceManager
    ArmController
    CalibrationManager
    Scanner
    TrajectoryPlanner
    ForceMonitor
    CleaningEvaluator
)