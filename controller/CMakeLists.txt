cmake_minimum_required(VERSION 3.5)
project(controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_definitions(-w)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(messages REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

include_directories(include ${rclcpp_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS})

# include yaml-cpp
include_directories(${YAML_INCLUDE_DIRS})
# include Erl
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Erl)
# include Eigen
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/Eigen-3.3.3)
# include ReflexxesTypeII (robot control lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/ReflexxesTypeII/include)
# include asio
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/asio/include)
# include customized modules
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules)
include_directories(${messages_INCLUDE_DIRS})

# library
find_library(REF_LIB ReflexxesTypeII 
             HINTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/ReflexxesTypeII/Linux/x64/release/lib/shared)
link_libraries(${REF_LIB} -lpthread)

# include master robot dependencies
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/force_dimension/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/force_dimension/lib/release/lin-x86_64-gcc/)

# find_library(USB_LIBS usb-1.0)
link_libraries(usb-1.0)

link_libraries(yaml-cpp)

# cpps
set(UTILS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/utils/utilities.cpp
              ${CMAKE_CURRENT_SOURCE_DIR}/modules/utils/robot_utilities.cpp
              )
set(IIWA_SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/app/iiwa_node.cpp
             ${CMAKE_CURRENT_SOURCE_DIR}/modules/iiwa/sunrise/kukasunrise.cpp
             ${CMAKE_CURRENT_SOURCE_DIR}/modules/iiwa/planner/taskspaceplanner.cpp
             ${CMAKE_CURRENT_SOURCE_DIR}/modules/iiwa/planner/iiwaplanner.cpp
             ${CMAKE_CURRENT_SOURCE_DIR}/modules/iiwa/planner/kukasunriseplanned.cpp
             ${CMAKE_CURRENT_SOURCE_DIR}/modules/iiwa/interface/iiwa_control.cpp
             )
      
SET(COMMANDER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/app/commander_node.cpp
                  ${CMAKE_CURRENT_SOURCE_DIR}/modules/omega/omega_control.cpp
                  )
              
set(KEYBOARD_SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/app/keyboard_control.cpp)


add_executable(iiwa_control ${CMAKE_CURRENT_SOURCE_DIR}/src/main_iiwacontrol.cpp ${IIWA_SRC} ${UTILS_SRC})
target_link_libraries(iiwa_control) # ${YAML_CPP_LIBRARIES}
ament_target_dependencies(iiwa_control rclcpp messages ament_index_cpp)

add_executable(commander ${CMAKE_CURRENT_SOURCE_DIR}/src/main_commander.cpp ${COMMANDER_SRC} ${UTILS_SRC})
target_link_libraries(commander dhd drd) # ${YAML_CPP_LIBRARIES}
ament_target_dependencies(commander rclcpp messages ament_index_cpp)

add_executable(keyboard_control ${CMAKE_CURRENT_SOURCE_DIR}/src/main_keyboard_control.cpp ${KEYBOARD_SRC})
ament_target_dependencies(keyboard_control rclcpp messages ament_index_cpp)

# Install C++ headers
install(DIRECTORY modules
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY dependencies
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

# Install Executable
install(TARGETS iiwa_control
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS commander
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS keyboard_control
        DESTINATION lib/${PROJECT_NAME})

# Install python scripts
install(PROGRAMS ${PROJECT_NAME}/position_recorder.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/trajectory_predictor.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/displayer.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY configs DESTINATION share/${PROJECT_NAME})

# Export package dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

ament_package()
