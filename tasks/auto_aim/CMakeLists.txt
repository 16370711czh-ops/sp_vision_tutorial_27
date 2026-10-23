add_library(auto_aim OBJECT
  armor.cpp solver.cpp target.cpp yolo.cpp yolos/yolov5.cpp
  task2_aimer.cpp task2_tracker.cpp task2_pipeline.cpp task2_config.cpp
  outpost_tracker.cpp task3_pipeline.cpp task3_config.cpp)
target_link_libraries(auto_aim PUBLIC task2_dependencies io openvino::runtime ${OpenCV_LIBS})
