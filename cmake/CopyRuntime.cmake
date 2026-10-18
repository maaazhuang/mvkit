cmake_minimum_required(VERSION 3.20)
if(POLICY CMP0207)
  cmake_policy(SET CMP0207 NEW)
endif()

# 构建时放到库旁；安装时由安装脚本指定 SDK bin，复用相同的依赖解析。
if(NOT MVKIT_RUNTIME_BINARIES)
  set(MVKIT_RUNTIME_BINARIES "${MVKIT_RUNTIME_BINARY}")
endif()
set(search_dirs)
foreach(binary IN LISTS MVKIT_RUNTIME_BINARIES)
  get_filename_component(binary_directory "${binary}" DIRECTORY)
  list(APPEND search_dirs "${binary_directory}")
endforeach()
if(MVKIT_RUNTIME_DESTINATION)
  set(destination "${MVKIT_RUNTIME_DESTINATION}")
else()
  list(GET search_dirs 0 destination)
endif()
file(MAKE_DIRECTORY "${destination}")
foreach(directory IN LISTS MVKIT_RUNTIME_DIRS)
  list(APPEND search_dirs "${directory}")
  get_filename_component(prefix "${directory}" DIRECTORY)
  list(APPEND search_dirs "${prefix}/bin")
endforeach()
list(REMOVE_DUPLICATES search_dirs)
file(GET_RUNTIME_DEPENDENCIES
  LIBRARIES ${MVKIT_RUNTIME_BINARIES}
  DIRECTORIES ${search_dirs}
  RESOLVED_DEPENDENCIES_VAR resolved
  UNRESOLVED_DEPENDENCIES_VAR unresolved
  PRE_EXCLUDE_REGEXES "api-ms-win-.*" "ext-ms-win-.*"
  POST_EXCLUDE_REGEXES ".*[Ss][Yy][Ss][Tt][Ee][Mm]32[/\\\\].*")
if(unresolved)
  message(FATAL_ERROR "Missing runtime dependencies: ${unresolved}. Add their bin directory to PATH or provide libraries under a common CMAKE_PREFIX_PATH.")
endif()
foreach(library IN LISTS resolved)
  execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${library}" "${destination}"
    COMMAND_ERROR_IS_FATAL ANY)
endforeach()
