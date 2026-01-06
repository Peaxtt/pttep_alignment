cmake_minimum_required(VERSION 3.8)
project(pttep_alignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. ค้นหา Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. สร้าง Custom Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalculateTransformation.srv"
  DEPENDENCIES geometry_msgs
)

# 3. สร้าง Alignment Node หลัก
add_executable(alignment_node src/alignment_node.cpp)
target_link_libraries(alignment_node Eigen3::Eigen)
ament_target_dependencies(alignment_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  std_srvs
)

# เชื่อมโยงกับ Custom Service
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(alignment_node "${cpp_typesupport_target}")

# 4. ติดตั้งเฉพาะไฟล์ที่ใช้งานจริง
install(TARGETS
  alignment_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
