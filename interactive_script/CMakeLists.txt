cmake_minimum_required(VERSION 3.5)
project(interactive_script)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sp_trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quad_common_utils REQUIRED)
find_package(interactive_markers REQUIRED)

find_package(Qt5 COMPONENTS Core Widgets WebEngineWidgets REQUIRED)
find_package(MiniLua REQUIRED)

set(interactive_script_plugin_SRCS
    src/interactive_script_plugin/interactive_script_plugin.cpp
    src/interactive_script_plugin/marker_interface.cpp
    src/interactive_script_plugin/quad_interface.cpp
    src/interactive_script_plugin/tf_interface.cpp
    src/interactive_script_plugin/builtins.cpp
    src/interactive_script_plugin/blocklybridge.cpp
    src/interactive_script_plugin/blocklywidget.cpp
    src/interactive_script_plugin/load_save_util.cpp
)

set(interactive_script_plugin_HDRS
    include/interactive_script/interactive_script_plugin.h
    include/interactive_script/marker_interface.h
    include/interactive_script/quad_interface.h
    include/interactive_script/tf_interface.h
    include/interactive_script/builtins.h
    include/interactive_script/blocklybridge.h
    include/interactive_script/blocklywidget.h
    include/interactive_script/load_save_util.h
)

set(interactive_script_plugin_UIS
    src/interactive_script_plugin/interactive_script_plugin.ui
)

set(interactive_script_plugin_INCLUDE_DIRECTORIES
    include
    ${rclcpp_INCLUDE_DIRS}
    ${ament_index_cpp_INCLUDE_DIRS}
    ${rqt_gui_cpp_INCLUDE_DIRS}
    ${quad_common_utils_INCLUDE_DIRS}
    ${interactive_markers_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5WebEngineWidgets_INCLUDE_DIRS}
    ${MiniLua_INCLUDE_DIRS}
    ${sp_trajectory_msgs_INCLUDE_DIRS}
)

qt5_wrap_cpp(interactive_script_plugin_MOCS ${interactive_script_plugin_HDRS})
qt5_wrap_ui(interactive_script_plugin_UIS_H ${interactive_script_plugin_UIS})

include_directories(
  ${interactive_script_plugin_INCLUDE_DIRECTORIES}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Build component
add_library(${PROJECT_NAME} SHARED
   ${interactive_script_plugin_SRCS}
   ${interactive_script_plugin_MOCS}
   ${interactive_script_plugin_UIS_H}
)

# target_compile_definitions(interactive_script_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${ament_index_cpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${tf2_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${quad_common_utils_LIBRARIES}
    ${rqt_gui_LIBRARIES}
    ${rqt_gui_cpp_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    ${sp_trajectory_msgs_LIBRARIES}
    Qt5::Widgets Qt5::Core Qt5::WebEngineWidgets
    MiniLua
)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(PROGRAMS scripts/interactive_script
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
