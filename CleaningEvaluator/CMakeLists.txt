CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)
project(CleaningEvaluator)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

# Convert relative paths to absolute paths
get_filename_component(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

SET (OpenCV_DIR "${ROOT_DIR}/Utils/yolov8-onnx-cpp/opencv/opencv/build/x64/vc16/lib")  # opencv lib root
SET (OpenCV_BIN_DIR "${ROOT_DIR}/Utils/yolov8-onnx-cpp/opencv/opencv/build/x64/vc16/bin")  #opencv bin root

SET (OpenCV_DEBUG_DLL_FILENAME opencv_world480d.dll)  # change filenames
SET (OpenCV_RELEASE_DLL_FILENAME opencv_world480.dll)  # change filenames

SET (ONNXRUNTIME_DIR "${ROOT_DIR}/Utils/yolov8-onnx-cpp/onnxruntime")  # onnxruntime root

SET (DETECT_INCLUDE "${ROOT_DIR}/Utils/yolov8-onnx-cpp/include")  
SET (DETECT_SRC "${ROOT_DIR}/Utils/yolov8-onnx-cpp/src")  

file(GLOB_RECURSE DETECT_SOURCES "${DETECT_SRC}/*.cpp")

FIND_PACKAGE(OpenCV REQUIRED)

add_executable(CleaningEvaluator
    CleaningEvaluator.cpp
    ${DETECT_SOURCES}
)

target_include_directories(CleaningEvaluator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ONNXRUNTIME_DIR}/include
    ${DETECT_INCLUDE}
)

target_compile_features(CleaningEvaluator PRIVATE cxx_std_17)

TARGET_LINK_LIBRARIES(CleaningEvaluator ${OpenCV_LIBS})

TARGET_LINK_LIBRARIES(CleaningEvaluator "${ONNXRUNTIME_DIR}/lib/onnxruntime.lib")

# Copy onnxruntime dll
add_custom_command(TARGET CleaningEvaluator POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ONNXRUNTIME_DIR}/lib/onnxruntime.dll"
        "$<TARGET_FILE_DIR:CleaningEvaluator>"
        )

# Copy OpenCV debug DLL to output directory after build
add_custom_command(TARGET CleaningEvaluator POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OpenCV_BIN_DIR}/${OpenCV_DEBUG_DLL_FILENAME}"
        "$<TARGET_FILE_DIR:CleaningEvaluator>"
        )

# Copy OpenCV release DLL to output directory after build
add_custom_command(TARGET CleaningEvaluator POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OpenCV_BIN_DIR}/${OpenCV_RELEASE_DLL_FILENAME}"
        "$<TARGET_FILE_DIR:CleaningEvaluator>"
        )

# # 设置导出宏
# target_compile_definitions(CleaningEvaluator PRIVATE
#     $<$<CONFIG:Debug>:CLEANING_EVALUATOR_DEBUG>
#     $<$<CONFIG:Release>:CLEANING_EVALUATOR_RELEASE>
# )