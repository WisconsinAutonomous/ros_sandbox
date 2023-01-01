cmake_minimum_required(VERSION 3.8)
project(cpp_image_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies    
find_package(ament_cmake_auto REQUIRED)    
ament_auto_find_build_dependencies()    
     
ament_auto_generate_code()    
     
ament_auto_add_executable(image_processor_node src/image_processor_node.cpp)    
     
ament_auto_package(    
    CONFIG_EXTRAS "cpp_image_processor-extras.cmake"    
)    
     
# ament_package() 


     
  
