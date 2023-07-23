cmake_minimum_required(VERSION 3.2)
project(helloworld)
set(CMAKE_CXX_STANDARD 11)
# The basic idea of the line below is to reduce the size of the binary files
# ref: https://www.redhat.com/en/blog/linkers-warnings-about-executable-stacks-and-segments
set(CMAKE_CXX_FLAGS "-static -Os -ffunction-sections -fdata-sections -fno-exceptions -Wl,--gc-sections,-s")
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/helloworld.cpp ${CMAKE_CURRENT_SOURCE_DIR}/PrimeChecker.cpp)

enable_testing()
add_executable(${PROJECT_NAME}-Runner TestPrimeChecker.cpp ${CMAKE_CURRENT_SOURCE_DIR}/PrimeChecker.cpp)
add_test(NAME ${PROJECT_NAME}-Runner COMMAND ${PROJECT_NAME}-Runner)
