add_library(ArmController SHARED
    ArmController.cpp
)

target_include_directories(ArmController PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# 设置编译选项
target_compile_options(ArmController PRIVATE /W4 /WX)

# 设置导出宏
target_compile_definitions(ArmController PRIVATE
    $<$<CONFIG:Debug>:ARM_CONTROLLER_DEBUG>
    $<$<CONFIG:Release>:ARM_CONTROLLER_RELEASE>
) 