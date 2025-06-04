add_library(SystemStateMachine SHARED
    SystemStateMachine.cpp
)

target_include_directories(SystemStateMachine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# 设置编译选项
target_compile_options(SystemStateMachine PRIVATE /W4 /WX)

# 设置导出宏
target_compile_definitions(SystemStateMachine PRIVATE
    $<$<CONFIG:Debug>:SYSTEM_STATE_MACHINE_DEBUG>
    $<$<CONFIG:Release>:SYSTEM_STATE_MACHINE_RELEASE>
) 