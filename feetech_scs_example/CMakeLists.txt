cmake_minimum_required(VERSION 3.8)
project(feetech_scs_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TARGET ping)
ament_auto_add_executable(${TARGET} ./src/${TARGET}.cpp)

set(TARGET write_pos)
ament_auto_add_executable(${TARGET} ./src/${TARGET}.cpp)

set(TARGET write_only)
ament_auto_add_executable(${TARGET} ./src/no_api/${TARGET}.cpp)

set(TARGET read_only)
ament_auto_add_executable(${TARGET} ./src/no_api/${TARGET}.cpp)

set(TARGET read_pos)
ament_auto_add_executable(${TARGET} ./src/${TARGET}.cpp)

set(TARGET read_spd)
ament_auto_add_executable(${TARGET} ./src/${TARGET}.cpp)

set(TARGET write_spd)
ament_auto_add_executable(${TARGET} ./src/${TARGET}.cpp)

set(TARGET set_pos)
ament_auto_add_executable(${TARGET} ./src/${TARGET}.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
