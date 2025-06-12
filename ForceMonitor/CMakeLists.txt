set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(ForceSix_ROOT "${CMAKE_SOURCE_DIR}/Utils/console_demo/include")
# set(ForceSix_LIB "${CMAKE_SOURCE_DIR}/Utils/console_demo/lib/windows_amd64/lib/kw-cpp-libd")

set(Base_ROOT "D:/Usmiles/UsmileRobotCpp")

set(ForceSix_ROOT "${Base_ROOT}/Utils/console_demo/include")
set(ForceSix_LIB "${Base_ROOT}/Utils/console_demo/lib/windows_amd64/lib/kw-cpp-libd")


add_executable(ForceMonitor
    ForceMonitor.cpp
)

# add_library(ForceMonitor SHARED
#     ForceMonitor.cpp
# )


target_include_directories(ForceMonitor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${ForceSix_ROOT}
)

target_link_libraries(ForceMonitor PUBLIC ${ForceSix_LIB})

# 设置导出宏
target_compile_definitions(ForceMonitor PRIVATE
    $<$<CONFIG:Debug>:FORCE_MONITOR_DEBUG>
    $<$<CONFIG:Release>:FORCE_MONITOR_RELEASE>
) 
