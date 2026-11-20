cmake_minimum_required(VERSION 3.16)
project(test_robot_mujoco)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(test_robot REQUIRED)
find_package(test_robot_moveit_config REQUIRED)
execute_process(COMMAND ${Python3_EXECUTABLE} -c
  "import pathlib,mujoco; print(pathlib.Path(mujoco.__file__).parent)"
  OUTPUT_VARIABLE MUJOCO_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE MUJOCO_RESULT)
if(NOT MUJOCO_RESULT EQUAL 0)
  message(FATAL_ERROR "MuJoCo Python package required: python3 -m pip install 'mujoco>=3.3,<4'")
endif()
file(GLOB MUJOCO_LIBRARIES "${MUJOCO_DIR}/libmujoco.so.*")
list(LENGTH MUJOCO_LIBRARIES MUJOCO_LIBRARY_COUNT)
if(NOT MUJOCO_LIBRARY_COUNT EQUAL 1)
  message(FATAL_ERROR "Expected one libmujoco.so in ${MUJOCO_DIR}")
endif()
add_library(mujoco_hardware SHARED src/mujoco_system.cpp)
target_include_directories(mujoco_hardware PRIVATE "${MUJOCO_DIR}/include")
target_link_libraries(mujoco_hardware ${MUJOCO_LIBRARIES})
ament_target_dependencies(mujoco_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs)
set_target_properties(mujoco_hardware PROPERTIES INSTALL_RPATH "${MUJOCO_DIR}")
pluginlib_export_plugin_description_file(hardware_interface mujoco_hardware.xml)
get_filename_component(DESCRIPTION_SHARE "${test_robot_DIR}/.." ABSOLUTE)
get_filename_component(MOVEIT_SHARE "${test_robot_moveit_config_DIR}/.." ABSOLUTE)
file(GLOB_RECURSE MODEL_INPUTS "${DESCRIPTION_SHARE}/urdf/*" "${DESCRIPTION_SHARE}/meshes/*")
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/model/test_robot.xml"
  COMMAND ${CMAKE_COMMAND} -E rm -rf "${CMAKE_CURRENT_BINARY_DIR}/model"
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_model.py"
    --description "${DESCRIPTION_SHARE}" --moveit "${MOVEIT_SHARE}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/model"
  DEPENDS scripts/generate_model.py ${MODEL_INPUTS} "${MOVEIT_SHARE}/config/test_robot.srdf"
  VERBATIM)
add_custom_target(mujoco_model ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/model/test_robot.xml")
install(TARGETS mujoco_hardware LIBRARY DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/model" DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/viewer.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/joint_telemetry.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md requirements.txt DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  add_executable(test_hardware test/test_hardware.cpp)
  ament_target_dependencies(test_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  add_test(NAME hardware_integration COMMAND test_hardware "${CMAKE_CURRENT_BINARY_DIR}/model/test_robot.xml")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(model_physics test/test_model.py
    ENV "MUJOCO_TEST_MODEL=${CMAKE_CURRENT_BINARY_DIR}/model/test_robot.xml")
  ament_add_pytest_test(viewer_telemetry test/test_telemetry.py
    ENV "MUJOCO_TEST_MODEL=${CMAKE_CURRENT_BINARY_DIR}/model/test_robot.xml")
endif()
ament_package()
