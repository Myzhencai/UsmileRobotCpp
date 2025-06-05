cmake_minimum_required(VERSION 3.10)
project(CodroidApi)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Boost 路径设置
set(BOOST_ROOT "D:/DrJ/boost_1_88_0")
set(NLOHMANN_ROOT "D:/DrJ/nlohmann")

# 查找 Boost 库
find_package(Boost 1.88 REQUIRED COMPONENTS system thread)

# 输出 Boost 信息
message("Boost version: ${Boost_VERSION_STRING}")
message("Boost include dir: ${Boost_INCLUDE_DIRS}")
message("Boost library dir: ${Boost_LIBRARY_DIRS}")

add_executable(CodroidApi Main.cpp)

target_compile_options(CodroidApi PRIVATE /bigobj)

# 添加头文件搜索路径
target_include_directories(CodroidApi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${NLOHMANN_ROOT}
)

# 链接 Boost 库
target_link_libraries(CodroidApi PRIVATE ${Boost_LIBRARIES})


 
