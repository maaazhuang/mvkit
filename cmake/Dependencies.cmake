if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
include(ExternalProject)
include(FetchContent)

option(MVKIT_USE_SYSTEM_LIBS "Try installed dependencies before source fallback" ON)
option(MVKIT_OFFLINE "Never download missing dependency sources" OFF)
set(MVKIT_DEPENDENCY_ROOT "${PROJECT_SOURCE_DIR}/3rdparty")

# 固定版本和归档校验；手工解压目录应使用同一版本并保留原始许可。
function(mvkit_dependency_source name out)
  if(name STREQUAL "jpeg")
    set(url "https://codeload.github.com/libjpeg-turbo/libjpeg-turbo/tar.gz/refs/tags/3.1.4.1")
    set(archive "libjpeg-turbo-3.1.4.1.tar.gz")
    set(hash a7da42b640377c2a9a9665e2c4b0ea60cd5599afb48c2521e6df0c9dc9d15a25)
  elseif(name STREQUAL "png")
    set(url "https://codeload.github.com/pnggroup/libpng/tar.gz/refs/tags/v1.6.58")
    set(archive "libpng-1.6.58.tar.gz")
    set(hash a9d4df463d36a6e5f9c29bd6f4967312d17e996c1854f3511f833924eb1993cf)
  elseif(name STREQUAL "tiff")
    set(url "https://download.osgeo.org/libtiff/tiff-4.7.2.tar.gz")
    set(archive "tiff-4.7.2.tar.gz")
    set(hash 672bd7d10aee4606171afb864f3570b83340f6a33e2c186dc0512f7145ffdf6a)
  elseif(name STREQUAL "zlib")
    set(url "https://zlib.net/zlib-1.3.2.tar.gz")
    set(archive "zlib-1.3.2.tar.gz")
    set(hash bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16)
  else()
    message(FATAL_ERROR "Unknown dependency: ${name}")
  endif()
  set(source "${MVKIT_DEPENDENCY_ROOT}/${name}")
  if(EXISTS "${source}/CMakeLists.txt")
    set(${out} "${source}" PARENT_SCOPE)
    return()
  endif()
  file(GLOB source_contents "${source}/*")
  if(source_contents)
    message(FATAL_ERROR "${source} must contain the upstream CMakeLists.txt directly; check the extracted directory layout")
  endif()
  set(archive_path "${MVKIT_DEPENDENCY_ROOT}/downloads/${archive}")
  if(EXISTS "${archive_path}")
    set(url "${archive_path}")
  elseif(MVKIT_OFFLINE)
    message(FATAL_ERROR "Offline dependency ${name} is missing. Extract ${archive} into ${source}, or place the archive at ${archive_path}")
  endif()
  # 只获取源码，不将不支持 add_subdirectory 的上游工程加入主构建。
  FetchContent_Declare(mvkit_source_${name}
    URL "${url}" URL_HASH "SHA256=${hash}"
    TLS_VERIFY TRUE
    DOWNLOAD_NAME "${archive}"
    DOWNLOAD_DIR "${MVKIT_DEPENDENCY_ROOT}/downloads"
    SOURCE_DIR "${source}" SOURCE_SUBDIR _mvkit_download_only_)
  FetchContent_MakeAvailable(mvkit_source_${name})
  if(NOT EXISTS "${source}/CMakeLists.txt")
    message(FATAL_ERROR "Dependency ${name} did not provide ${source}/CMakeLists.txt")
  endif()
  set(${out} "${source}" PARENT_SCOPE)
endfunction()

if(MVKIT_USE_SYSTEM_LIBS)
  find_package(JPEG QUIET)
  find_package(PNG QUIET)
  find_package(TIFF QUIET)
  find_package(ZLIB QUIET)
  foreach(dependency JPEG PNG TIFF ZLIB)
    if(TARGET ${dependency}::${dependency})
      message(STATUS "MVKit dependency ${dependency}: installed library")
    endif()
  endforeach()
endif()

# 系统库缺哪一项才编译哪一项，已安装库和源码库可以混合使用。
if(NOT TARGET JPEG::JPEG OR NOT TARGET PNG::PNG OR NOT TARGET TIFF::TIFF)
  # libjpeg-turbo 不支持 add_subdirectory，因此统一以独立上游 CMake 构建静态库。
  set(dependency_prefix "${CMAKE_BINARY_DIR}/_deps/install")
  set(dependency_configs ${CMAKE_CONFIGURATION_TYPES})
  if(NOT dependency_configs)
    set(dependency_configs ${CMAKE_BUILD_TYPE})
  endif()
  foreach(config IN LISTS dependency_configs)
    file(MAKE_DIRECTORY "${dependency_prefix}/${config}/include")
  endforeach()
  set(dependency_args
    "-DCMAKE_INSTALL_PREFIX:PATH=${dependency_prefix}/$<CONFIG>"
    "-DCMAKE_BUILD_TYPE:STRING=$<CONFIG>"
    "-DCMAKE_INSTALL_LIBDIR:STRING=lib"
    "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON"
    "-DBUILD_SHARED_LIBS:BOOL=OFF"
    "-DBUILD_TESTING:BOOL=OFF")
  foreach(setting CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_TOOLCHAIN_FILE
                  CMAKE_SYSROOT CMAKE_OSX_ARCHITECTURES CMAKE_OSX_DEPLOYMENT_TARGET
                  CMAKE_MSVC_RUNTIME_LIBRARY)
    if(DEFINED ${setting} AND NOT "${${setting}}" STREQUAL "")
      list(APPEND dependency_args "-D${setting}=${${setting}}")
    endif()
  endforeach()
  function(mvkit_static_dependency name target release_library debug_library)
    mvkit_dependency_source(${name} source)
    message(STATUS "MVKit dependency ${name}: source (${source})")
    ExternalProject_Add(mvkit_dep_${name}
      SOURCE_DIR "${source}"
      DOWNLOAD_COMMAND ""
      PREFIX "${CMAKE_BINARY_DIR}/_deps/${name}"
      CMAKE_ARGS ${dependency_args} ${ARGN}
      BUILD_COMMAND "${CMAKE_COMMAND}" --build <BINARY_DIR> --config $<CONFIG> --parallel 4
      INSTALL_COMMAND "${CMAKE_COMMAND}" --install <BINARY_DIR> --config $<CONFIG>
      BUILD_BYPRODUCTS "${dependency_prefix}/$<CONFIG>/lib/$<IF:$<CONFIG:Debug>,${debug_library},${release_library}>"
      EXCLUDE_FROM_ALL TRUE)
    add_library(${target} STATIC IMPORTED GLOBAL)
    foreach(config IN LISTS dependency_configs)
      string(TOUPPER "${config}" upper)
      if(config STREQUAL "Debug")
        set(library "${debug_library}")
      else()
        set(library "${release_library}")
      endif()
      set_property(TARGET ${target} APPEND PROPERTY IMPORTED_CONFIGURATIONS ${config})
      set_property(TARGET ${target} PROPERTY IMPORTED_LOCATION_${upper} "${dependency_prefix}/${config}/lib/${library}")
    endforeach()
    set_property(TARGET ${target} PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${dependency_prefix}/$<CONFIG>/include")
    add_dependencies(${target} mvkit_dep_${name})
  endfunction()
  if(MSVC)
    set(zlib_release zs.lib)
    set(zlib_debug zsd.lib)
    set(jpeg_library jpeg-static.lib)
    set(png_library libpng16_static.lib)
    set(tiff_library tiff.lib)
  else()
    if(WIN32)
      set(zlib_release libzs.a)
      set(zlib_debug libzsd.a)
    else()
      set(zlib_release libz.a)
      set(zlib_debug libz.a)
    endif()
    set(jpeg_library libjpeg.a)
    set(png_library libpng16.a)
    set(tiff_library libtiff.a)
  endif()
  if((NOT TARGET PNG::PNG OR NOT TARGET TIFF::TIFF) AND NOT TARGET ZLIB::ZLIB)
    mvkit_static_dependency(zlib ZLIB::ZLIB
      ${zlib_release} ${zlib_debug}
      -DZLIB_BUILD_SHARED=OFF -DZLIB_BUILD_STATIC=ON -DZLIB_BUILD_TESTING=OFF)
  endif()
  find_program(MVKIT_NASM_EXECUTABLE nasm)
  set(jpeg_simd OFF)
  if(MVKIT_NASM_EXECUTABLE OR CMAKE_SYSTEM_PROCESSOR MATCHES "[Aa][Rr][Mm]|aarch64" OR CMAKE_GENERATOR_PLATFORM MATCHES "ARM")
    set(jpeg_simd ON)
  endif()
  if(NOT TARGET JPEG::JPEG)
    mvkit_static_dependency(jpeg JPEG::JPEG
      ${jpeg_library} ${jpeg_library}
      -DENABLE_SHARED=OFF -DENABLE_STATIC=ON -DWITH_CRT_DLL=ON
      -DWITH_TURBOJPEG=OFF -DWITH_TOOLS=OFF -DWITH_TESTS=OFF -DWITH_SIMD=${jpeg_simd})
  endif()
  if(TARGET ZLIB::ZLIB)
    set(zlib_path "$<TARGET_FILE:ZLIB::ZLIB>")
    set(zlib_include "$<TARGET_PROPERTY:ZLIB::ZLIB,INTERFACE_INCLUDE_DIRECTORIES>")
  endif()
  set(jpeg_path "$<TARGET_FILE:JPEG::JPEG>")
  set(jpeg_include "$<TARGET_PROPERTY:JPEG::JPEG,INTERFACE_INCLUDE_DIRECTORIES>")
  if(NOT TARGET PNG::PNG)
    mvkit_static_dependency(png PNG::PNG
      ${png_library} ${png_library}
      -DPNG_SHARED=OFF -DPNG_STATIC=ON -DPNG_TESTS=OFF -DPNG_TOOLS=OFF
      -DPNG_DEBUG_POSTFIX:STRING=
      "-DZLIB_INCLUDE_DIR:PATH=${zlib_include}"
      "-DZLIB_LIBRARY:FILEPATH=${zlib_path}")
    if(TARGET mvkit_dep_zlib)
      add_dependencies(mvkit_dep_png mvkit_dep_zlib)
    endif()
    set_property(TARGET PNG::PNG PROPERTY INTERFACE_LINK_LIBRARIES ZLIB::ZLIB)
  endif()
  if(NOT TARGET TIFF::TIFF)
    mvkit_static_dependency(tiff TIFF::TIFF
      ${tiff_library} ${tiff_library}
      -Dtiff-static=ON -Dtiff-tools=OFF -Dtiff-tests=OFF -Dtiff-contrib=OFF
      -Dtiff-docs=OFF -Dtiff-install=ON -Dtiff-cxx=OFF
      -Djpeg=ON -Djpeg-prefer-standard=ON -Dzlib=ON
      -Djbig=OFF -Dlerc=OFF -Dlzma=OFF -Dzstd=OFF -Dwebp=OFF
      -DCMAKE_DISABLE_FIND_PACKAGE_JBIG=ON -DCMAKE_DISABLE_FIND_PACKAGE_LERC=ON
      -DCMAKE_DISABLE_FIND_PACKAGE_liblzma=ON -DCMAKE_DISABLE_FIND_PACKAGE_ZSTD=ON
      -DCMAKE_DISABLE_FIND_PACKAGE_WebP=ON
      "-DZLIB_INCLUDE_DIR:PATH=${zlib_include}"
      "-DZLIB_LIBRARY:FILEPATH=${zlib_path}"
      "-DJPEG_INCLUDE_DIR:PATH=${jpeg_include}"
      "-DJPEG_LIBRARY:FILEPATH=${jpeg_path}")
    foreach(dependency zlib jpeg)
      if(TARGET mvkit_dep_${dependency})
        add_dependencies(mvkit_dep_tiff mvkit_dep_${dependency})
      endif()
    endforeach()
    set_property(TARGET TIFF::TIFF PROPERTY INTERFACE_LINK_LIBRARIES "JPEG::JPEG;ZLIB::ZLIB")
  endif()
  # 只安装实际编入 MVKit 的源码依赖许可；系统库由其安装方维护许可。
  foreach(dependency jpeg png tiff zlib)
    if(TARGET mvkit_dep_${dependency})
      set(source "${MVKIT_DEPENDENCY_ROOT}/${dependency}")
      if(dependency STREQUAL "jpeg")
        set(licenses "${source}/LICENSE.md" "${source}/README.ijg")
      elseif(dependency STREQUAL "png")
        set(licenses "${source}/LICENSE" "${source}/README")
      elseif(dependency STREQUAL "tiff")
        set(licenses "${source}/LICENSE.md")
      else()
        set(licenses "${source}/LICENSE" "${source}/README")
      endif()
      install(FILES ${licenses} DESTINATION "${CMAKE_INSTALL_DATADIR}/MVKit/third-party/${dependency}")
    endif()
  endforeach()
endif()

