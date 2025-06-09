cmake_minimum_required(VERSION 3.12)

project(Test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TARGET_NAME kw-test)
add_executable(${TARGET_NAME} main.cpp)

target_link_libraries(${TARGET_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/lib/windows_amd64/lib/kw-cpp-libd.lib)
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)