add_library(ForceMonitor SHARED
    ForceMonitor.cpp
)

target_include_directories(ForceMonitor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# 设置编译选项
target_compile_options(ForceMonitor PRIVATE /W4 /WX)

# 设置导出宏
target_compile_definitions(ForceMonitor PRIVATE
    $<$<CONFIG:Debug>:FORCE_MONITOR_DEBUG>
    $<$<CONFIG:Release>:FORCE_MONITOR_RELEASE>
) 