# 按模块目录收集实现；新增/删除 .cpp 时由 CONFIGURE_DEPENDS 触发重新配置。
# SIMD 文件先排除，随后按目标架构加入，并保留各文件原有的编译选项。
function(mvkit_collect_sources out)
  set(sources)
  foreach(directory IN LISTS ARGN)
    file(GLOB_RECURSE module_sources CONFIGURE_DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/${directory}/*.cpp")
    list(APPEND sources ${module_sources})
  endforeach()
  list(FILTER sources EXCLUDE REGEX "_avx2[.]cpp$")
  set(${out} ${sources} PARENT_SCOPE)
endfunction()

# 核心对象与私有运行时。
mvkit_collect_sources(mvkit_core_sources src/core src/runtime)
# 这两项由后端选项控制，不能随普通 runtime 源码无条件加入。
list(FILTER mvkit_core_sources EXCLUDE REGEX
  "/(builtin_pool_resource|builtin_executor)[.]cpp$")
add_library(mvkit_core STATIC ${mvkit_core_sources})
if(MVKIT_ENABLE_MEMORY_POOL)
  target_sources(mvkit_core PRIVATE
    src/runtime/memory/builtin_pool_resource.cpp)
endif()
if(MVKIT_ENABLE_PARALLEL)
  target_sources(mvkit_core PRIVATE
    src/runtime/parallel/builtin_executor.cpp)
endif()
if(MVKIT_ENABLE_AVX2_KERNEL)
  target_sources(mvkit_core PRIVATE src/core/run_copy_avx2.cpp)
  target_compile_definitions(mvkit_core PRIVATE MVKIT_CORE_HAS_AVX2=1)
  if(MSVC)
    set_source_files_properties(src/core/run_copy_avx2.cpp PROPERTIES COMPILE_OPTIONS /arch:AVX2)
  else()
    set_source_files_properties(src/core/run_copy_avx2.cpp PROPERTIES COMPILE_OPTIONS -mavx2)
  endif()
else()
  target_compile_definitions(mvkit_core PRIVATE MVKIT_CORE_HAS_AVX2=0)
endif()
target_include_directories(mvkit_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}/generated
)
target_link_libraries(mvkit_core PRIVATE Threads::Threads)
target_compile_features(mvkit_core PRIVATE cxx_std_20)
set_target_properties(mvkit_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 各算法模块独立链接，只声明各自需要的依赖和特殊编译选项。
mvkit_collect_sources(mvkit_geometry_sources src/geometry)
add_library(mvkit_geometry STATIC ${mvkit_geometry_sources})
target_include_directories(mvkit_geometry PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mvkit_geometry PRIVATE mvkit_core)
target_compile_features(mvkit_geometry PRIVATE cxx_std_20)
set_target_properties(mvkit_geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MVKIT_ENABLE_AVX2_KERNEL)
  # 几何坐标仍用 Double；AVX2 单独编译且禁止乘加融合。
  target_sources(mvkit_geometry PRIVATE src/geometry/analytic2d_avx2.cpp)
  target_compile_definitions(mvkit_geometry PRIVATE MVKIT_GEOMETRY_HAS_AVX2=1)
  if(MSVC)
    set_source_files_properties(src/geometry/analytic2d_avx2.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:strict")
  else()
    set_source_files_properties(src/geometry/analytic2d_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
  endif()
else()
  target_compile_definitions(mvkit_geometry PRIVATE MVKIT_GEOMETRY_HAS_AVX2=0)
endif()

mvkit_collect_sources(mvkit_calibration_sources src/calibration)
add_library(mvkit_calibration STATIC ${mvkit_calibration_sources})
target_include_directories(mvkit_calibration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mvkit_calibration PRIVATE mvkit_core mvkit_geometry)
if(MVKIT_USE_EIGEN)
  if(NOT EXISTS "${MVKIT_3RDPARTY_DIR}/eigen/Eigen/Dense")
    message(FATAL_ERROR
      "MVKIT_USE_EIGEN requires 3rdparty/eigen/Eigen/Dense; "
      "the include root must resolve <Eigen/Dense>")
  endif()
  target_include_directories(mvkit_calibration SYSTEM PRIVATE ${MVKIT_3RDPARTY_DIR}/eigen)
  target_compile_definitions(mvkit_calibration PRIVATE MVKIT_USE_EIGEN=1)
endif()
target_compile_features(mvkit_calibration PRIVATE cxx_std_20)
set_target_properties(mvkit_calibration PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MVKIT_ENABLE_AVX2_KERNEL)
  target_sources(mvkit_calibration PRIVATE src/calibration/estimation_avx2.cpp)
  target_compile_definitions(mvkit_calibration PRIVATE MVKIT_CALIBRATION_HAS_AVX2=1)
  if(MSVC)
    set_source_files_properties(src/calibration/estimation_avx2.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:strict")
  else()
    set_source_files_properties(src/calibration/estimation_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
  endif()
else()
  target_compile_definitions(mvkit_calibration PRIVATE MVKIT_CALIBRATION_HAS_AVX2=0)
endif()

mvkit_collect_sources(mvkit_reconstruction_sources src/reconstruction)
add_library(mvkit_reconstruction STATIC ${mvkit_reconstruction_sources})
target_include_directories(mvkit_reconstruction PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mvkit_reconstruction PRIVATE mvkit_core mvkit_calibration)
target_compile_features(mvkit_reconstruction PRIVATE cxx_std_20)
set_target_properties(mvkit_reconstruction PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(MVKIT_ENABLE_AVX2_KERNEL)
  file(GLOB_RECURSE reconstruction_avx2_sources CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/reconstruction/*_avx2.cpp")
  target_sources(mvkit_reconstruction PRIVATE ${reconstruction_avx2_sources})
  target_compile_definitions(mvkit_reconstruction PRIVATE MVKIT_RECONSTRUCTION_HAS_AVX2=1)
  if(MSVC)
    set_source_files_properties(${reconstruction_avx2_sources} PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:strict")
  else()
    set_source_files_properties(${reconstruction_avx2_sources} PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
  endif()
else()
  target_compile_definitions(mvkit_reconstruction PRIVATE MVKIT_RECONSTRUCTION_HAS_AVX2=0)
endif()

mvkit_collect_sources(mvkit_imageproc_sources src/imageproc)
add_library(mvkit_imageproc STATIC ${mvkit_imageproc_sources})
target_include_directories(mvkit_imageproc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mvkit_imageproc PRIVATE mvkit_core)
target_compile_features(mvkit_imageproc PRIVATE cxx_std_20)
set_target_properties(mvkit_imageproc PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MVKIT_ENABLE_AVX2_KERNEL)
  target_sources(mvkit_imageproc PRIVATE src/imageproc/filters_avx2.cpp src/imageproc/arithmetic_avx2.cpp)
  target_compile_definitions(mvkit_imageproc PRIVATE MVKIT_FILTER_HAS_AVX2=1)
  if(MSVC)
    set_source_files_properties(src/imageproc/filters_avx2.cpp PROPERTIES COMPILE_OPTIONS /arch:AVX2)
    set_source_files_properties(src/imageproc/arithmetic_avx2.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:strict")
  else()
    set_source_files_properties(src/imageproc/filters_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
    set_source_files_properties(src/imageproc/arithmetic_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
  endif()
else()
  target_compile_definitions(mvkit_imageproc PRIVATE MVKIT_FILTER_HAS_AVX2=0)
endif()

mvkit_collect_sources(mvkit_region_sources src/region)
add_library(mvkit_region STATIC ${mvkit_region_sources})
target_include_directories(mvkit_region PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mvkit_region PRIVATE mvkit_core)
target_compile_features(mvkit_region PRIVATE cxx_std_20)
set_target_properties(mvkit_region PROPERTIES POSITION_INDEPENDENT_CODE ON)

mvkit_collect_sources(mvkit_xld_sources src/xld)
add_library(mvkit_xld STATIC ${mvkit_xld_sources})
target_include_directories(mvkit_xld PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mvkit_xld PRIVATE mvkit_core mvkit_region mvkit_imageproc)
target_compile_features(mvkit_xld PRIVATE cxx_std_20)
set_target_properties(mvkit_xld PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MVKIT_ENABLE_AVX2_KERNEL)
  target_sources(mvkit_xld PRIVATE src/xld/distance_avx2.cpp src/xld/fast_paths_avx2.cpp src/xld/affine_avx2.cpp)
  if(MSVC)
    set_source_files_properties(src/xld/fast_paths_avx2.cpp src/xld/affine_avx2.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:strict")
  else()
    set_source_files_properties(src/xld/fast_paths_avx2.cpp src/xld/affine_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
  endif()
  target_compile_definitions(mvkit_xld PRIVATE MVKIT_XLD_HAS_AVX2=1)
  if(MSVC)
    set_source_files_properties(src/xld/distance_avx2.cpp PROPERTIES COMPILE_OPTIONS /arch:AVX2)
  else()
    set_source_files_properties(src/xld/distance_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
  endif()
else()
  target_compile_definitions(mvkit_xld PRIVATE MVKIT_XLD_HAS_AVX2=0)
endif()

mvkit_collect_sources(mvkit_ncc_matching_sources src/ncc_matching)
add_library(mvkit_ncc_matching STATIC ${mvkit_ncc_matching_sources})
target_include_directories(mvkit_ncc_matching PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mvkit_ncc_matching PRIVATE mvkit_core mvkit_imageproc)
target_compile_features(mvkit_ncc_matching PRIVATE cxx_std_20)
set_target_properties(mvkit_ncc_matching PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MVKIT_ENABLE_AVX2_KERNEL)
  target_sources(mvkit_ncc_matching PRIVATE src/ncc_matching/kernels_avx2.cpp)
  target_compile_definitions(mvkit_ncc_matching PRIVATE MVKIT_NCC_HAS_AVX2=1)
  if(MSVC)
    set_source_files_properties(src/ncc_matching/kernels_avx2.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:strict")
  else()
    set_source_files_properties(src/ncc_matching/kernels_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
  endif()
else()
  target_compile_definitions(mvkit_ncc_matching PRIVATE MVKIT_NCC_HAS_AVX2=0)
endif()

mvkit_collect_sources(mvkit_shape_matching_sources src/shape_matching)
add_library(mvkit_shape_matching STATIC ${mvkit_shape_matching_sources})
if(MVKIT_ENABLE_AVX2_KERNEL)
  target_sources(mvkit_shape_matching PRIVATE src/shape_matching/gradient_avx2.cpp)
  target_compile_definitions(mvkit_shape_matching PRIVATE MVKIT_SHAPE_HAS_AVX2_KERNEL=1)
  if(MSVC)
    set_source_files_properties(src/shape_matching/gradient_avx2.cpp PROPERTIES
                                COMPILE_OPTIONS /arch:AVX2)
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set_source_files_properties(src/shape_matching/gradient_avx2.cpp PROPERTIES
                                COMPILE_OPTIONS -mavx2)
  endif()
else()
  target_compile_definitions(mvkit_shape_matching PRIVATE MVKIT_SHAPE_HAS_AVX2_KERNEL=0)
endif()
target_include_directories(mvkit_shape_matching PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mvkit_shape_matching PRIVATE mvkit_core mvkit_imageproc mvkit_xld)
target_compile_features(mvkit_shape_matching PRIVATE cxx_std_20)
set_target_properties(mvkit_shape_matching PROPERTIES POSITION_INDEPENDENT_CODE ON)

mvkit_collect_sources(mvkit_imageio_sources src/imageio)
add_library(mvkit_imageio STATIC ${mvkit_imageio_sources})
target_include_directories(mvkit_imageio PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mvkit_imageio PRIVATE mvkit_core JPEG::JPEG PNG::PNG TIFF::TIFF)
target_compile_features(mvkit_imageio PRIVATE cxx_std_20)
set_target_properties(mvkit_imageio PROPERTIES POSITION_INDEPENDENT_CODE ON)

# C 与 C++ 两层公开共享库；私有实现不作为安装目标导出。
mvkit_collect_sources(mvkit_sources src/c_api)
add_library(mvkit SHARED ${mvkit_sources})
target_include_directories(mvkit
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/c_api ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(mvkit PRIVATE mvkit_core mvkit_imageio mvkit_imageproc mvkit_region mvkit_xld mvkit_shape_matching mvkit_ncc_matching mvkit_geometry mvkit_calibration mvkit_reconstruction)
target_compile_features(mvkit PRIVATE cxx_std_20)
# 公开头包含 UTF-8 中文注释；导入 SDK 的 MSVC C/C++ 调用方使用相同编码。
target_compile_options(mvkit INTERFACE
  "$<$<COMPILE_LANG_AND_ID:C,MSVC>:/utf-8>"
  "$<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/utf-8>"
)
if(UNIX AND NOT APPLE)
  set(mvkit_version_script "${CMAKE_CURRENT_BINARY_DIR}/generated/mvkit.map")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/mvkit.map.in" "${mvkit_version_script}" @ONLY)
  target_link_options(mvkit PRIVATE "LINKER:--version-script=${mvkit_version_script}")
  set_property(TARGET mvkit APPEND PROPERTY LINK_DEPENDS "${mvkit_version_script}")
endif()
target_compile_definitions(mvkit
  PRIVATE
    MVKIT_BUILD_SHARED
    MVKIT_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    MVKIT_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    MVKIT_VERSION_PATCH=${PROJECT_VERSION_PATCH}
  INTERFACE
    MVKIT_USE_SHARED
)
set_target_properties(mvkit PROPERTIES
  OUTPUT_NAME mvkit
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME C
)
if(WIN32)
  get_filename_component(mvkit_linker_dir "${CMAKE_LINKER}" DIRECTORY)
  set(mvkit_tiff_runtime_target TIFF::TIFF)
  # 新版 libtiff Config 将 TIFF::TIFF 包装为 interface，实际库是 TIFF::tiff。
  if(TARGET TIFF::tiff)
    set(mvkit_tiff_runtime_target TIFF::tiff)
  endif()
  if(MSVC)
    set(mvkit_runtime_dependency_command "${mvkit_linker_dir}/dumpbin.exe")
    set(mvkit_runtime_dependency_tool dumpbin)
    set(mvkit_runtime_tool "-DCMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND=${mvkit_linker_dir}/dumpbin.exe")
  else()
    set(mvkit_runtime_dependency_command "${CMAKE_OBJDUMP}")
    set(mvkit_runtime_dependency_tool objdump)
    set(mvkit_runtime_tool "-DCMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND=${CMAKE_OBJDUMP}"
      "-DCMAKE_GET_RUNTIME_DEPENDENCIES_TOOL=objdump")
  endif()
  add_custom_command(TARGET mvkit POST_BUILD
    COMMAND ${CMAKE_COMMAND}
      "-DMVKIT_RUNTIME_BINARY=$<TARGET_FILE:mvkit>"
      "-DMVKIT_RUNTIME_DIRS=$<TARGET_FILE_DIR:JPEG::JPEG>;$<TARGET_FILE_DIR:PNG::PNG>;$<TARGET_FILE_DIR:${mvkit_tiff_runtime_target}>"
      ${mvkit_runtime_tool}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CopyRuntime.cmake
    VERBATIM)
endif()

mvkit_collect_sources(mvkitcpp_sources src/cpp_api)
add_library(mvkitcpp SHARED ${mvkitcpp_sources})
target_include_directories(mvkitcpp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp_api)
target_include_directories(mvkitcpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(mvkitcpp PUBLIC mvkit)
target_compile_features(mvkitcpp PRIVATE cxx_std_20 INTERFACE cxx_std_14)
target_compile_definitions(mvkitcpp
  PRIVATE
    MVKIT_CPP_BUILD_SHARED
  INTERFACE
    MVKIT_CPP_USE_SHARED
)
set_target_properties(mvkitcpp PROPERTIES
  OUTPUT_NAME mvkitcpp
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME CXX
)

add_library(mvkit::C ALIAS mvkit)
add_library(mvkit::CXX ALIAS mvkitcpp)

# 安装后的 C++ 库从同一目录查找 C 运行库，支持 SDK 前缀重定位。
if(APPLE)
  set_target_properties(mvkit mvkitcpp PROPERTIES INSTALL_RPATH "@loader_path")
elseif(UNIX)
  set_target_properties(mvkit mvkitcpp PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()
