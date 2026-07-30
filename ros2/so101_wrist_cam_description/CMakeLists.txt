cmake_minimum_required(VERSION 3.8)
project(so101_wrist_cam_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch worlds rviz config
  DESTINATION share/${PROJECT_NAME}
)

# demo_motion.py is executed by name from sim.launch.py (demo:=true), so it
# must land in lib/<pkg> where ros2 run looks, not in share/.
install(
  PROGRAMS scripts/demo_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
