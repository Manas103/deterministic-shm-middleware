cmake_minimum_required(VERSION 3.16)
project(deterministic_shm_middleware CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(Threads REQUIRED)

option(ENABLE_ASAN "Enable AddressSanitizer + UBSan" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

if(ENABLE_ASAN)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1)
  add_link_options(-fsanitize=address,undefined)
endif()

if(ENABLE_TSAN)
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g -O1)
  add_link_options(-fsanitize=thread)
endif()

# Everything a block computes, independent of how its messages move. Both
# transports (block_runner here, ros2_block_node in ros2/shmmw_ros2) compile
# exactly these sources.
set(SHMMW_CORE_SOURCES
  src/graph_config.cpp
  src/scenario.cpp
  src/av_model.cpp
  src/kernels.cpp
  src/block_engine.cpp
  src/reference.cpp)

add_library(shmmw_core ${SHMMW_CORE_SOURCES})
target_include_directories(shmmw_core PUBLIC include)
target_link_libraries(shmmw_core PUBLIC Threads::Threads)
target_compile_options(shmmw_core PUBLIC -Wall -Wextra)

add_executable(block_runner src/block_runner.cpp)
target_link_libraries(block_runner PRIVATE shmmw_core rt)

add_executable(graph_launcher src/graph_launcher.cpp)
target_link_libraries(graph_launcher PRIVATE shmmw_core rt)

add_executable(monitor src/monitor.cpp)
target_link_libraries(monitor PRIVATE shmmw_core rt)

add_executable(snooper src/snooper.cpp)
target_link_libraries(snooper PRIVATE shmmw_core rt)

add_executable(sim_reference src/sim_reference.cpp)
target_link_libraries(sim_reference PRIVATE shmmw_core)

enable_testing()

add_executable(check_ring_buffer tests/test_ring_buffer.cpp)
target_link_libraries(check_ring_buffer PRIVATE shmmw_core)
add_test(NAME check_ring_buffer COMMAND check_ring_buffer)

add_executable(check_snooper tests/test_snooper.cpp)
target_link_libraries(check_snooper PRIVATE shmmw_core rt)
add_test(NAME check_snooper COMMAND check_snooper)

add_executable(check_sha256 tests/test_sha256.cpp)
target_link_libraries(check_sha256 PRIVATE shmmw_core)
add_test(NAME check_sha256 COMMAND check_sha256)

add_executable(check_av_model tests/test_av_model.cpp)
target_link_libraries(check_av_model PRIVATE shmmw_core)
add_test(NAME check_av_model COMMAND check_av_model)

add_executable(check_scenario tests/test_scenario.cpp)
target_link_libraries(check_scenario PRIVATE shmmw_core)
target_compile_definitions(check_scenario PRIVATE SHMMW_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME check_scenario COMMAND check_scenario)

add_executable(check_engine tests/test_engine.cpp)
target_link_libraries(check_engine PRIVATE shmmw_core)
target_compile_definitions(check_engine PRIVATE SHMMW_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME check_engine COMMAND check_engine)
