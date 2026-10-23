# Tests compile the same sources used by auto_aim/tools, without camera or OpenVINO stubs.
add_library(task2_estimation STATIC
  ../tasks/auto_aim/target.cpp ../tasks/auto_aim/task2_aimer.cpp
  ../tasks/auto_aim/task2_tracker.cpp ../tasks/auto_aim/task2_pipeline.cpp
  ../tasks/auto_aim/task2_config.cpp ../tools/extended_kalman_filter.cpp
  ../tools/math_tools.cpp ../tools/trajectory.cpp ../tools/logger.cpp ../tools/crc.cpp)
target_link_libraries(task2_estimation PUBLIC task2_dependencies)
add_executable(task2_regression task_2_test.cpp)
target_link_libraries(task2_regression PRIVATE task2_estimation)
add_test(NAME task2_legacy_regression COMMAND task2_regression)
add_executable(task2_acceptance task2_acceptance.cpp)
target_link_libraries(task2_acceptance PRIVATE task2_estimation)
target_include_directories(task2_acceptance PRIVATE "${PROJECT_SOURCE_DIR}/io/serial/include")
foreach(suite IN ITEMS model tracking pipeline timing config)
  add_test(NAME task2_${suite} COMMAND task2_acceptance ${suite} "${PROJECT_SOURCE_DIR}/configs/task2.yaml")
endforeach()
if(OpenCV_FOUND)
  add_executable(task2_solver_roundtrip task2_solver_roundtrip.cpp
    ../tasks/auto_aim/solver.cpp ../tasks/auto_aim/armor.cpp)
  target_link_libraries(task2_solver_roundtrip PRIVATE task2_estimation ${OpenCV_LIBS})
  add_test(NAME task2_solver_roundtrip COMMAND task2_solver_roundtrip "${PROJECT_SOURCE_DIR}/configs/task2.yaml")
else()
  message(STATUS "Solver/PnP runtime test NOT enabled: OpenCV runtime libraries unavailable.")
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/task_1_angles_test.cpp")
  add_executable(task1_compatibility task_1_angles_test.cpp)
  target_link_libraries(task1_compatibility PRIVATE task2_estimation)
  add_test(NAME task1_compatibility COMMAND task1_compatibility)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/task_3_test.cpp" AND EXISTS "${PROJECT_SOURCE_DIR}/tasks/auto_aim/outpost_tracker.cpp")
  add_executable(task3_compatibility task_3_test.cpp ../tasks/auto_aim/outpost_tracker.cpp)
  target_link_libraries(task3_compatibility PRIVATE task2_estimation)
  add_test(NAME task3_compatibility COMMAND task3_compatibility)
endif()
