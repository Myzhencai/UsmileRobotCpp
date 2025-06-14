cmake_minimum_required(VERSION 3.10)
project(ScannerTest)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置项目根路径
set(Base_ROOT "D:/Usmiles/UsmileRobotCpp")

# 头文件目录
set(Scanner_INCLUDE_DIR "${Base_ROOT}/Utils/TJST3DScannerApiSDK/include")

# lib 文件所在目录
set(Scanner_LIB_DIR "${Base_ROOT}/Utils/TJST3DScannerApiSDK/Lib")

# 只使用库名，不加 .lib 后缀
set(Scanner_LIB_NAME "TJST3DScannerApi")

# 包含头文件路径
include_directories(${Scanner_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加 lib 文件路径
link_directories(${Scanner_LIB_DIR})

message(STATUS "Include path: ${Scanner_INCLUDE_DIR}")
message(STATUS "Lib path: ${Scanner_LIB_DIR}")
message(STATUS "Link lib: ${Scanner_LIB_NAME}")

# 可执行文件
add_executable(ScannerTest Scanner.cpp)

# 链接库（只写名称，不加路径、不加 .lib）
target_link_libraries(ScannerTest ${Scanner_LIB_NAME})





