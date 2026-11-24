cmake_minimum_required(VERSION 3.16)
project(swarm_mission_planner LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(swarm_planner STATIC
  src/core/grid_path_planner.cpp
  src/core/mission_planner.cpp
  src/core/cbs_coordinator.cpp
  src/core/task_metadata.cpp
  src/offline/offline_mission_planner.cpp
  src/offline/maps/map_bundle_loader.cpp
  src/offline/maps/multi_map_path_planner.cpp
  src/offline/coordination/multi_map_coordinator.cpp
  src/offline/coordination/route_reservations.cpp
  src/offline/coordination/safe_interval_scheduler.cpp)
target_compile_features(swarm_planner PUBLIC cxx_std_17)
target_compile_options(swarm_planner PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(swarm_planner PUBLIC include PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(swarm_planner PRIVATE Boost::boost yaml-cpp ${OpenCV_LIBS})

add_executable(swarm_planner_native src/main.cpp)
target_link_libraries(swarm_planner_native PRIVATE swarm_planner nlohmann_json::nlohmann_json)
target_compile_options(swarm_planner_native PRIVATE -Wall -Wextra -Wpedantic)

ament_python_install_package(${PROJECT_NAME})
install(TARGETS swarm_planner_native DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/swarm_mission_planner_node scripts/swarm_plan DESTINATION lib/${PROJECT_NAME})
install(FILES README.md TEST_LOG.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(planner_test test TIMEOUT 90
    ENV "SWARM_PLANNER_NATIVE=$<TARGET_FILE:swarm_planner_native>")
endif()

ament_package()
