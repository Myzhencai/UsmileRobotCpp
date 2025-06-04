add_library(CalibrationManager SHARED
    CalibrationManager.cpp
)

target_include_directories(CalibrationManager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# 设置编译选项
target_compile_options(CalibrationManager PRIVATE /W4 /WX)

# 设置导出宏
target_compile_definitions(CalibrationManager PRIVATE
    $<$<CONFIG:Debug>:CALIBRATION_MANAGER_DEBUG>
    $<$<CONFIG:Release>:CALIBRATION_MANAGER_RELEASE>
) 