cmake_minimum_required(VERSION 3.5)
project(octomap_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

find_package(PCL REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OCTOMAP REQUIRED)

# 新增：Boost 用于 websocketpp
find_package(Boost REQUIRED COMPONENTS system thread)

# 静态八叉图服务器
ament_auto_add_library(octomap_server_static SHARED
  src/octomap_server_static.cpp
)

target_link_libraries(octomap_server_static
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_include_directories(octomap_server_static
  PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
)

rclcpp_components_register_node(octomap_server_static
  PLUGIN "octomap_server::OctomapServerStatic"
  EXECUTABLE octomap_server_static_node
)

# 基础八叉图服务器
ament_auto_add_library(octomap_server SHARED
  src/octomap_server.cpp
)

target_link_libraries(octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${Boost_LIBRARIES}  # 新增：Boost
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenSSL REQUIRED openssl)
target_link_libraries(octomap_server ${OpenSSL_LIBRARIES})
target_include_directories(octomap_server PUBLIC ${OpenSSL_INCLUDE_DIRS})

target_compile_definitions(octomap_server PRIVATE ASIO_STANDALONE=1)
target_compile_options(octomap_server PRIVATE -Wno-template-id-cdtp-mismatch -Wno-non-template-friend)

# 新增：此目标的包含目录（PCL + Octomap + websocketpp），必须在 target_link_libraries 之后
target_include_directories(octomap_server
  PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/websocketpp" # 新增：websocketpp（确保目录存在）
)

rclcpp_components_register_node(octomap_server
  PLUGIN "octomap_server::OctomapServer"
  EXECUTABLE octomap_server_node
)

# 多层八叉树服务器
ament_auto_add_library(octomap_server_multilayer SHARED
  src/octomap_server.cpp
  src/octomap_server_multilayer.cpp
)

target_link_libraries(octomap_server_multilayer
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_include_directories(octomap_server_multilayer
  PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/websocketpp"
)

rclcpp_components_register_node(octomap_server_multilayer
  PLUGIN "octomap_server::OctomapServerMultilayer"
  EXECUTABLE octomap_server_multilayer_node
)

# 跟踪八叉图服务器
ament_auto_add_library(tracking_octomap_server SHARED
  src/octomap_server.cpp
  src/tracking_octomap_server.cpp
)

target_link_libraries(tracking_octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${Boost_LIBRARIES}
)

target_include_directories(tracking_octomap_server
  PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/websocketpp"
)

rclcpp_components_register_node(tracking_octomap_server
  PLUGIN "octomap_server::TrackingOctomapServer"
  EXECUTABLE tracking_octomap_server_node
)

# 彩色八叉图服务器
ament_auto_add_library(color_octomap_server SHARED
  src/octomap_server.cpp
)

target_link_libraries(color_octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_include_directories(color_octomap_server
  PUBLIC
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/websocketpp"
)

rclcpp_components_register_node(color_octomap_server
  PLUGIN "octomap_server::ColorOctomapServer"
  EXECUTABLE color_octomap_server_node
)

target_compile_definitions(color_octomap_server PUBLIC COLOR_OCTOMAP_SERVER)

# 八叉图保存器
ament_auto_add_library(octomap_saver SHARED
  src/octomap_saver.cpp
)

target_link_libraries(octomap_saver
  ${OCTOMAP_LIBRARIES}
)

rclcpp_components_register_node(octomap_saver
  PLUGIN "octomap_server::OctomapSaver"
  EXECUTABLE octomap_saver_node
)

# WS 测试程序（原 LCM 测试重命名）
add_executable(ws_client_test
  src/ws_client_test.cpp # 确保此文件存在，参考之前代码
)
target_link_libraries(ws_client_test
  ${OpenSSL_LIBRARIES}
  # ${Boost_LIBRARIES}
)


target_include_directories(ws_client_test PUBLIC ${OpenSSL_INCLUDE_DIRS})
target_compile_definitions(ws_client_test PRIVATE ASIO_STANDALONE=1)
target_compile_options(ws_client_test PRIVATE -Wno-deprecated-declarations -Wno-template-id-cdtp-mismatch)


install(TARGETS ws_client_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  params
)

install(PROGRAMS
  scripts/octomap_eraser_cli.py
  DESTINATION lib/${PROJECT_NAME}
)
