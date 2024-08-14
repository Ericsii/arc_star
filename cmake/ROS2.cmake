# -------- Find thirdparty library -------- # ROS packages
set(ros_dependencies ament_cmake rclcpp rclcpp_components event_camera_msgs
                     event_camera_codecs)

foreach(dependency ${ros_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

# Other packages
find_package(Eigen3 REQUIRED)

set(other_dependencies Eigen3::Eigen)

# ------------- Build library ------------- #
add_library(${PROJECT_NAME} SHARED "arc_star/src/arc_star_detector.cc"
                                   "src/arc_star_node.cpp")
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE arc_star/include)
target_link_libraries(${PROJECT_NAME} ${other_dependencies})
ament_target_dependencies(${PROJECT_NAME} ${ros_dependencies})

# ---------------- Install --------------- #
install(DIRECTORY include/
  DESTINATION include
)

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${ros_dependencies})

ament_package()