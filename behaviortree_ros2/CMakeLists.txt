cmake_minimum_required(VERSION 3.16)
project(behaviortree_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    btcpp_ros2_interfaces
    generate_parameter_library
    )

find_package(ament_cmake REQUIRED)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
    find_package(${PACKAGE} REQUIRED )
endforeach()


generate_parameter_library(
    bt_executor_parameters
    src/bt_executor_parameters.yaml)

add_library(${PROJECT_NAME}
    src/bt_ros2.cpp
    src/bt_utils.cpp
    src/tree_execution_server.cpp )

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}  bt_executor_parameters)


######################################################
# INSTALL

install(DIRECTORY include/  DESTINATION include/)

ament_export_targets(${PROJECT_NAME}Targets  HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

install(
  TARGETS ${PROJECT_NAME} bt_executor_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING) # 由构建参数控制是否生成模拟 Action 回归程序。
  enable_testing() # 注册下方 add_test 定义的 CTest 检查。
  add_executable(test_action_lifecycle test/test_action_lifecycle.cpp) # 编译现有 Sleep 接口的模拟检查。
  target_include_directories(test_action_lifecycle PRIVATE include) # 测试直接包含本次修改的模板头文件。
  ament_target_dependencies(test_action_lifecycle ${THIS_PACKAGE_DEPS}) # 复用包内已有依赖。
  add_test(NAME action_lifecycle COMMAND test_action_lifecycle) # 一个程序覆盖反馈、取消及生命周期。
  set_tests_properties(action_lifecycle PROPERTIES
    TIMEOUT 30 # 运行超过 30 秒时由 CTest 结束检查。
    ENVIRONMENT "ROS_DOMAIN_ID=188;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs") # 使用独立域和本地日志目录。
endif()

ament_package()
