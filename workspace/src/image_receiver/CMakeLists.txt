cmake_minimum_required(VERSION 3.8)    
project(image_receiver)    
     
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")    
  add_compile_options(-Wall -Wextra -Wpedantic)    
endif()    
     
# find dependencies    
find_package(ament_cmake_auto REQUIRED)    
ament_auto_find_build_dependencies()    
     
ament_auto_generate_code()    
     
# rclcpp_components_register_nodes(${PROJECT_NAME} "image_receiver::image_receiver_node")
# ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)    
     
ament_auto_package(    
    CONFIG_EXTRAS "image_receiver-extras.cmake"    
)    
     
# ament_package()   
