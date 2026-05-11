cmake_minimum_required(VERSION 3.5)
project(agv_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake が最低限必要
find_package(ament_cmake REQUIRED)

# ※ Python ノード（rclpy）やメッセージ型（sensor_msgs／geometry_msgs）は
#   実際のビルドステップで C++ をコンパイルしないので、ここでは find_package は不要です。
#   必要な依存は package.xml の <exec_depend> で管理します。

# install 以下の設定
install(DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
