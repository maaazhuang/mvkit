set(mvkit_demo_names image_processing region_morphology xld_edges shape_matching ncc_matching)
foreach(demo IN LISTS mvkit_demo_names)
  add_executable(mvkit_demo_${demo} ${demo}.cpp)
  target_link_libraries(mvkit_demo_${demo} PRIVATE mvkit::CXX)
  set_target_properties(mvkit_demo_${demo} PROPERTIES
    CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
endforeach()

# 合并演示同时输出中间结果和五组三联图，便于复现示例效果。
add_executable(mvkit_industrial_demo industrial_demo.cpp)
target_link_libraries(mvkit_industrial_demo PRIVATE mvkit::CXX)
set_target_properties(mvkit_industrial_demo PROPERTIES
  CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
