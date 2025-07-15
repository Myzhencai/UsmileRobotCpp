# 设置CMake最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(force_control)

# 设置C++17标准
add_definitions(-std=c++17)

# 查找并配置Eigen3库
# find_package(Eigen3 3.4 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

set(Eigen3_DIR "D:/force_control/thirdparty/eigen-3.4.0")
include_directories(${Eigen3_DIR})

# 添加头文件目录
include_directories(
  include
)

# 查找yaml-cpp和RobotUtilities库
# find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)
include_directories("D:/force_control/thirdparty/yaml-cpp/include")
include_directories("D:/force_control/cpplibrary/build/install/include")

link_directories("D:/force_control/thirdparty/yaml-cpp/lib")

# find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

# 这里的意思是：查找名为Utilities的库，并用变量RUT来表示它（即RUT是Utilities库的别名）
find_library(RUT Utilities HINTS D:/force_control/cpplibrary/build/install/lib/RobotUtilities)
# 之后可以用${RUT}来代表Utilities库

# 创建共享库
# add_library(FORCE_CONTROLLERS SHARED src/admittance_controller.cpp src/config_deserialize.cpp)
add_executable(FORCE_CONTROLLERS demo.cpp src/admittance_controller.cpp src/config_deserialize.cpp)
  # target_link_libraries(FORCE_CONTROLLERS ${RUT} ${YAMLLib})
  target_link_libraries(FORCE_CONTROLLERS ${RUT} yaml-cpp)

  

# # 安装头文件
# install(DIRECTORY include/
#   DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
# )

# # 安装库文件
# install(TARGETS FORCE_CONTROLLERS
#   DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
# )