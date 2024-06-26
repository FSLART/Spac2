cmake_minimum_required(VERSION 3.8)
project(spac2_0)

set(library_name spac2_0)
set(lib_srcs src/spac_node.cpp src/fmath.cpp src/target.cpp)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lart_msgs REQUIRED)

include_directories(include)

add_library(${library_name} ${lib_srcs})

ament_target_dependencies(${library_name} rclcpp std_msgs nav_msgs geometry_msgs lart_msgs tf2)


if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	ament_add_gtest(spac2_0_test test/tst_target.cpp)
	ament_target_dependencies(spac2_0_test rclcpp std_msgs nav_msgs geometry_msgs lart_msgs tf2)
	target_link_libraries(spac2_0_test spac2_0)
  else()
	ament_uncrustify(${library_name} ${lib_srcs})
endif()

add_executable(spac_node src/spac_node.cpp src/target.cpp src/fmath.cpp)

ament_target_dependencies(
  spac_node
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "std_msgs"
  "lart_msgs"
)

target_link_libraries(spac_node ${library_name})

install(TARGETS
  spac_node
  DESTINATION lib/${PROJECT_NAME}
)

# add config folder
install(DIRECTORY ../config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ../launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
