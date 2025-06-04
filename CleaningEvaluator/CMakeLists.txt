add_library(CleaningEvaluator SHARED
    CleaningEvaluator.cpp
)

target_include_directories(CleaningEvaluator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# 设置编译选项
target_compile_options(CleaningEvaluator PRIVATE /W4 /WX)

# 设置导出宏
target_compile_definitions(CleaningEvaluator PRIVATE
    $<$<CONFIG:Debug>:CLEANING_EVALUATOR_DEBUG>
    $<$<CONFIG:Release>:CLEANING_EVALUATOR_RELEASE>
) 