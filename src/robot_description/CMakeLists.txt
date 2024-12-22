cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Dodaj flagi kompilatora dla ostrzeżeń
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Znajdź pakiety wymagane przez ROS2
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Instalacja folderu launch (dla plików launch)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Instalacja folderu urdf (dla plików URDF)
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Instalacja folderu meshes (dla plików STL)
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Jeśli włączono testowanie, znajdź zależności do automatycznych testów lint
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Pakiet ROS2
ament_package()
