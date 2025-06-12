set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Base_ROOT "D:/Usmiles/UsmileRobotCpp")

set(Scanner_ROOT "${Base_ROOT}/Utils/TJST3DScannerApiSDK/include")
set(Scanner_LIB "${Base_ROOT}/Utils/TJST3DScannerApiSDK/Lib/TJST3DScannerApi.lib")


# add_library(Scanner SHARED
#     Scanner.cpp
# )

add_executable(Scanner Scanner.cpp
)

target_include_directories(Scanner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${Scanner_ROOT}
)

target_link_libraries(Scanner PUBLIC ${Scanner_LIB})

# 设置导出宏
target_compile_definitions(Scanner PRIVATE
    $<$<CONFIG:Debug>:SCANNER_DEBUG>
    $<$<CONFIG:Release>:SCANNER_RELEASE>
) 