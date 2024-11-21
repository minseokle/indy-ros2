cmake_minimum_required(VERSION 3.10)
project(indy_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
set(_GRPC_GRPCPP gRPC::grpc++)

# Include directories for indydcp3_lib
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${gRPC_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/indy_driver/neuromeka_cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/indy_driver/neuromeka_cpp/proto)

# Source files for C++ nodes
set(NRMK_CPP_SOURCES
  indy_driver/neuromeka_cpp/indydcp3.cpp
  indy_driver/neuromeka_cpp/proto/common_msgs.pb.cc
  indy_driver/neuromeka_cpp/proto/config_msgs.pb.cc
  indy_driver/neuromeka_cpp/proto/control_msgs.pb.cc
  indy_driver/neuromeka_cpp/proto/device_msgs.pb.cc
  indy_driver/neuromeka_cpp/proto/ethercat_msgs.pb.cc
  indy_driver/neuromeka_cpp/proto/moby_msgs.pb.cc
  indy_driver/neuromeka_cpp/proto/rtde_msgs.pb.cc
  indy_driver/neuromeka_cpp/proto/hri_msgs.pb.cc
  indy_driver/neuromeka_cpp/proto/shared_msgs.pb.cc

  indy_driver/neuromeka_cpp/proto/common_msgs.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/config_msgs.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/control_msgs.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/device_msgs.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/ethercat_msgs.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/moby.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/rtde.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/hri.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/shared_msgs.grpc.pb.cc

  indy_driver/neuromeka_cpp/proto/config.pb.cc
  indy_driver/neuromeka_cpp/proto/control.pb.cc
  indy_driver/neuromeka_cpp/proto/device.pb.cc
  indy_driver/neuromeka_cpp/proto/ethercat.pb.cc
  indy_driver/neuromeka_cpp/proto/eyetask.pb.cc
  indy_driver/neuromeka_cpp/proto/indyeye.pb.cc
  indy_driver/neuromeka_cpp/proto/moby.pb.cc
  indy_driver/neuromeka_cpp/proto/rtde.pb.cc
  indy_driver/neuromeka_cpp/proto/cri.pb.cc
  indy_driver/neuromeka_cpp/proto/hri.pb.cc

  indy_driver/neuromeka_cpp/proto/config.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/control.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/device.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/ethercat.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/eyetask.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/indyeye.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/moby.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/rtde.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/cri.grpc.pb.cc
  indy_driver/neuromeka_cpp/proto/hri.grpc.pb.cc
)

# Add executables
add_executable(indy_driver_cpp indy_driver/indy_driver_cpp.cpp ${NRMK_CPP_SOURCES})

# Link Protobuf and gRPC static libraries
target_link_libraries(indy_driver_cpp
  ${Protobuf_LIBRARIES} 
  ${gRPC_LIBRARIES}
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
)

# Use ament dependencies for ROS2 dependencies
ament_target_dependencies(indy_driver_cpp
  rclcpp
  std_msgs
  sensor_msgs
)

# Install C++ executables
install(TARGETS indy_driver_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(DIRECTORY
  launch controller
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  ${PROJECT_NAME}/indy_driver.py
  ${PROJECT_NAME}/servo_joy_input.py
  ${PROJECT_NAME}/servo_keyboard_input.py
  ${PROJECT_NAME}/indy_define.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
