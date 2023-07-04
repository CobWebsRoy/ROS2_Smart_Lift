cmake_minimum_required(VERSION 3.8)
project(lift_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#Added:
find_package(rosidl_default_generators REQUIRED)

#Added:
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/LiftRequests.msg"
	"msg/LiftStates.msg"
	"msg/ButtonPress.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
