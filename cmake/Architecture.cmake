# 将生成器/工具链目标映射为 MVKit 架构标签。
# 不使用指针宽度或宿主处理器单独决定 AVX2 TU。

function(mvkit_map_platform_name name out_var)
  string(TOUPPER "${name}" _name)
  if(_name STREQUAL "X64" OR _name STREQUAL "WIN64" OR _name STREQUAL "X86_64" OR
     _name STREQUAL "AMD64")
    set(${out_var} x64 PARENT_SCOPE)
  elseif(_name STREQUAL "WIN32" OR _name STREQUAL "X86" OR _name STREQUAL "I386")
    set(${out_var} x86 PARENT_SCOPE)
  elseif(_name STREQUAL "ARM64" OR _name STREQUAL "AARCH64")
    set(${out_var} arm64 PARENT_SCOPE)
  elseif(_name STREQUAL "ARM64EC")
    set(${out_var} arm64ec PARENT_SCOPE)
  elseif(_name STREQUAL "ARM" OR _name STREQUAL "ARMV7")
    set(${out_var} arm PARENT_SCOPE)
  else()
    set(${out_var} unknown PARENT_SCOPE)
  endif()
endfunction()

function(mvkit_detect_target_architecture)
  set(_arch unknown)
  set(_source unspecified)

  if(CMAKE_GENERATOR MATCHES "Visual Studio")
    set(_plat "${CMAKE_VS_PLATFORM_NAME}")
    if(_plat STREQUAL "")
      set(_plat "${CMAKE_GENERATOR_PLATFORM}")
    endif()
    mvkit_map_platform_name("${_plat}" _arch)
    set(_source "vs_platform:${_plat}")
  else()
    set(_cpu "${CMAKE_SYSTEM_PROCESSOR}")
    if(DEFINED CMAKE_OSX_ARCHITECTURES AND CMAKE_OSX_ARCHITECTURES)
      list(LENGTH CMAKE_OSX_ARCHITECTURES _osx_n)
      if(_osx_n EQUAL 1)
        set(_cpu "${CMAKE_OSX_ARCHITECTURES}")
      else()
        set(_arch unknown)
        set(_source "multi_osx_arch")
      endif()
    endif()
    if(NOT _source STREQUAL "multi_osx_arch")
      mvkit_map_platform_name("${_cpu}" _arch)
      set(_source "system_processor:${_cpu}")
    endif()
  endif()

  include(CheckCXXSourceCompiles)
  unset(MVKIT_COMPILES_AS_X64 CACHE)
  unset(MVKIT_COMPILES_AS_ARM64 CACHE)
  check_cxx_source_compiles("
#if defined(_M_X64) || defined(__x86_64__)
int main(){return 0;}
#else
#error not x64
#endif
" MVKIT_COMPILES_AS_X64)
  check_cxx_source_compiles("
#if defined(_M_ARM64) || defined(__aarch64__)
int main(){return 0;}
#else
#error not arm64
#endif
" MVKIT_COMPILES_AS_ARM64)

  set(_probe unknown)
  if(MVKIT_COMPILES_AS_X64)
    set(_probe x64)
  elseif(MVKIT_COMPILES_AS_ARM64)
    set(_probe arm64)
  endif()

  set(MVKIT_TARGET_ARCH "${_arch}" PARENT_SCOPE)
  set(MVKIT_TARGET_ARCH_SOURCE "${_source}" PARENT_SCOPE)
  set(MVKIT_COMPILE_ARCH_PROBE "${_probe}" PARENT_SCOPE)

  set(_enable_avx2 OFF)
  if(_arch STREQUAL "x64" AND _probe STREQUAL "x64")
    set(_enable_avx2 ON)
  elseif(_arch STREQUAL "x64" AND NOT _probe STREQUAL "x64")
    message(WARNING
      "MVKit: VS/toolset platform is x64 but compile macros are '${_probe}'; "
      "disabling AVX2 translation units")
  elseif(_arch STREQUAL "arm64" AND _probe STREQUAL "x64")
    message(WARNING
      "MVKit: platform is ARM64 but compile macros look like x64; "
      "disabling specialized SIMD translation units")
  endif()
  set(MVKIT_ENABLE_AVX2_KERNEL ${_enable_avx2} PARENT_SCOPE)
endfunction()
