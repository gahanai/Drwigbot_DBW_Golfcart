cmake_minimum_required(VERSION 3.8)
project(radar_node)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(THREADS_HAVE_PTHREAD_ARG 1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(Threads REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

list(APPEND ALL_LIBS
        ${SERIAL_LDFLAGS}
         ${CMAKE_THREAD_LIBS_INIT}
        )
list(APPEND INCLUDE_DIRS
        include
        )
find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/ClusterObj.msg"
  "msg/DetectedObj.msg"
  "msg/TrackedObj.msg"
  "msg/ArrayClusterObj.msg"
  "msg/ArrayDetectedObj.msg"
  "msg/ArrayTrackedObj.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
 )
ament_export_dependencies(rosidl_default_runtime)
# find_package(ros_node_interface REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${INCLUDE_DIRS})
include_directories(include/)
file(GLOB_RECURSE AllHeaders ${PROJECT_SOURCE_DIR}/*.hpp)

add_executable(${PROJECT_NAME}_node
  src/drwig_parser.cpp
  src/publisher_node.cpp
  ${AllHeaders})


target_include_directories(${PROJECT_NAME}_node PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node  ${ALL_LIBS})
ament_target_dependencies(${PROJECT_NAME}_node  rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")