cmake_minimum_required(VERSION 3.8)
project(rclcpp_gtest_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  example_interfaces
)

# find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES
  src/add_two_ints_client.cpp
)

# Creating library for test linking
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES} )
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
###

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server ${PROJECT_DEPENDENCIES})

add_executable(client src/client_main.cpp src/add_two_ints_client.cpp)
ament_target_dependencies(client ${PROJECT_DEPENDENCIES})

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
endif()

ament_package()
