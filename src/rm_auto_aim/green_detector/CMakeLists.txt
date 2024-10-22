cmake_minimum_required(VERSION 3.10)
project(green_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)
add_definitions(-O3)


## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::GreenDetectorNode
  EXECUTABLE green_detector_node
)

#############
## Testing ##
#############

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
)
