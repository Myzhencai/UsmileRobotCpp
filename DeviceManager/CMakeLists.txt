add_library(DeviceManager SHARED
    DeviceManager.cpp
)

target_include_directories(DeviceManager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# 设置编译选项
target_compile_options(DeviceManager PRIVATE /W4 /WX)

# 设置导出宏
target_compile_definitions(DeviceManager PRIVATE
    $<$<CONFIG:Debug>:DEVICE_MANAGER_DEBUG>
    $<$<CONFIG:Release>:DEVICE_MANAGER_RELEASE>
) 