cmake_minimum_required(VERSION 3.10)
project(MyTest)

set(CMAKE_CXX_STANDARD 14)

find_package(GTest REQUIRED)

set(EXAMPLE_SOURCE_FILES
    test_global.cpp
    test_stack.cpp
    test_single.cpp
)

set(TEST_SOURCE_FILES
    test_rpm_speed_conversion.cpp
)

set(TEST_SOURCE_FILES1
    test_decode_rpm.cpp
)

add_executable(MyStackTest ${EXAMPLE_SOURCE_FILES})
add_executable(MyCANReceiveTest ${TEST_SOURCE_FILES})
add_executable(MyCANReceiveTest1 ${TEST_SOURCE_FILES1})

include_directories(
  ${GTEST_INCLUDE_DIRS}
)

target_link_libraries(MyStackTest
  ${GTEST_LIBRARIES}
  pthread
)

target_include_directories(MyCANReceiveTest PRIVATE
  ${GTEST_INCLUDE_DIRS}
)

target_link_libraries(MyCANReceiveTest
  ${GTEST_LIBRARIES}
  pthread
)

target_link_libraries(MyCANReceiveTest1
  ${GTEST_LIBRARIES}
  pthread
)

