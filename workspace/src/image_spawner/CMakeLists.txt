cmake_minimum_required(VERSION 3.8)
project(image_spawner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_generate_code()

ament_auto_add_executable(image_spawner_node src/image_spawner_node.cpp)

ament_auto_package(
	CONFIG_EXTRAS "image_spawner-extras.cmake"
	)

#ament_package()
