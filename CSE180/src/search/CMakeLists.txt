cmake_minimum_required(VERSION 3.8)
project(search)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# MRTP course package. Its installed CMake config exports the static library
# libnavigator.a as the imported target navigation::navigator
# (MRTP/install/navigation/share/navigation/cmake/navigatorlibExport.cmake).
find_package(navigation REQUIRED)

add_executable(search src/search.cpp)

ament_target_dependencies(search
  rclcpp
)

# Link the course Navigator through its exported target, not by path.
target_link_libraries(search
  navigation::navigator
)

install(TARGETS
  search
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
