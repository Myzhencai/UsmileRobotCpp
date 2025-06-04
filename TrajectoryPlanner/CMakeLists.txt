add_library(TrajectoryPlanner SHARED
    TrajectoryPlanner.cpp
)

target_include_directories(TrajectoryPlanner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# 设置编译选项
target_compile_options(TrajectoryPlanner PRIVATE /W4 /WX)

# 设置导出宏
target_compile_definitions(TrajectoryPlanner PRIVATE
    $<$<CONFIG:Debug>:TRAJECTORY_PLANNER_DEBUG>
    $<$<CONFIG:Release>:TRAJECTORY_PLANNER_RELEASE>
) 