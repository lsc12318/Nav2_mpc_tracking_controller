cmake_minimum_required(VERSION 3.8)
project(mpc_tracking_controller)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)
include_directories(/usr/local/include   /usr/include/eigen3 include/ ${catkin_INCLUDE_DIRS})


set(dependencies
        rclcpp
        geometry_msgs
        nav2_costmap_2d
        pluginlib
        nav_msgs
        nav2_util
        nav2_core
        tf2

        )

set(library_name mpc_tracking_controller)


add_library(${library_name} SHARED
        src/mpc_plannner.cpp  src/mpc_tracking_controller.cpp src/regulated_pure_pursuit_controller.cpp)
target_link_libraries(${library_name}  ipopt)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${library_name}
        ${dependencies}
        )


install(TARGETS ${library_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )


install(DIRECTORY include/
        DESTINATION include/
        )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   set(ament_cmake_copyright_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
#   add_subdirectory(test)
# endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

#add_executable(mpc_tracking_controller src/mpc_plannner.cpp  src/mpc_tracking_controller.cpp)


pluginlib_export_plugin_description_file(nav2_core mpc_tracking_controller.xml)



ament_package()
