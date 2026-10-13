cmake_minimum_required(VERSION 3.16)
project(ar4_my_planners)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ompl REQUIRED)   # ompl::ompl（CMake target 名，stage_1_ompl_3.md §10）

# 本机（ROS Jazzy，apt 的 moveit_planners_ompl 2.12.4）不能用它的导出 target：
# 头文件装在 include/moveit_planners/ 下，而 target 声明的 INTERFACE_INCLUDE_DIRECTORIES
# 是不存在的 include/moveit_planners_ompl/，链接它 CMake Generate 直接失败。
# 因此头文件目录和 .so 都按文件定位；ompl 的头/库走正常的 ompl::ompl target。
find_path(MOVEIT_PLANNERS_OMPL_INCLUDE_DIR
  NAMES moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.hpp
  PATH_SUFFIXES moveit_planners moveit_planners_ompl)
if(NOT MOVEIT_PLANNERS_OMPL_INCLUDE_DIR)
  message(FATAL_ERROR
    "未找到 moveit/ompl_interface 头文件（搜索后缀 moveit_planners / moveit_planners_ompl）")
endif()
find_library(MOVEIT_OMPL_INTERFACE_LIBRARY NAMES moveit_ompl_interface)
if(NOT MOVEIT_OMPL_INTERFACE_LIBRARY)
  message(FATAL_ERROR "未找到 libmoveit_ompl_interface.so")
endif()

add_library(ar4_my_planners_plugin SHARED
  src/planner_manager.cpp
  src/planning_context.cpp
)

# ── BiRRTStar 算法库（birrtstar设计_v2.md §11）──────────────────────────
# 独立 target、只链 ompl::ompl：从构建系统层面强制算法层与 ROS/MoveIt 隔离，
# 二维 harness（Phase B）与 MoveIt 插件（Phase C）共用这一份代码。
add_library(ar4_birrtstar STATIC
  src/birrtstar.cpp
)
target_include_directories(ar4_birrtstar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ar4_birrtstar PUBLIC ompl::ompl)
target_compile_features(ar4_birrtstar PUBLIC cxx_std_17)
# 要链进插件 .so，必须位置无关；不改变依赖面（仍只链 ompl::ompl）
set_target_properties(ar4_birrtstar PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 二维验证程序（stage_1_ompl_3.md §5 Phase B）：无 ROS/MoveIt 依赖
add_executable(birrtstar_2d src/birrtstar_2d.cpp)
target_link_libraries(birrtstar_2d PRIVATE ar4_birrtstar)
target_compile_features(birrtstar_2d PRIVATE cxx_std_17)

target_include_directories(ar4_my_planners_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ar4_my_planners_plugin PRIVATE
  ${MOVEIT_PLANNERS_OMPL_INCLUDE_DIR})

ament_target_dependencies(ar4_my_planners_plugin
  moveit_core moveit_ros_planning pluginlib rclcpp moveit_msgs)

# 链接 BiRRTStar 静态库（只链 ompl::ompl，见上）与 libmoveit_ompl_interface.so
target_link_libraries(ar4_my_planners_plugin
  ar4_birrtstar
  ompl::ompl
  ${MOVEIT_OMPL_INTERFACE_LIBRARY})

pluginlib_export_plugin_description_file(moveit_core plugin_description.xml)

install(TARGETS ar4_my_planners_plugin LIBRARY DESTINATION lib)
install(TARGETS ar4_birrtstar ARCHIVE DESTINATION lib)
install(TARGETS birrtstar_2d RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(moveit_core moveit_ros_planning pluginlib rclcpp moveit_msgs ompl)

ament_package()
