add_library(Scanner SHARED
    Scanner.cpp
)

target_include_directories(Scanner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# 设置编译选项
target_compile_options(Scanner PRIVATE /W4 /WX)

# 设置导出宏
target_compile_definitions(Scanner PRIVATE
    $<$<CONFIG:Debug>:SCANNER_DEBUG>
    $<$<CONFIG:Release>:SCANNER_RELEASE>
) 