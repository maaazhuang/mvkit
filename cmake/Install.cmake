# SDK 安装公开头、共享库、可重定位的 CMake package 及许可材料。
install(TARGETS mvkit mvkitcpp
  EXPORT MVKitTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/mvkit DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
file(RELATIVE_PATH MVKIT_SDK_INCLUDE_RELATIVE
  "${CMAKE_INSTALL_FULL_DATADIR}/MVKit" "${CMAKE_INSTALL_FULL_INCLUDEDIR}")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/README.md" mvkit_project_readme)
string(FIND "${mvkit_project_readme}" "## 第三方组件与许可" mvkit_notices_begin)
if(mvkit_notices_begin EQUAL -1)
  message(FATAL_ERROR "README.md must contain the third-party components and licenses section")
endif()
string(SUBSTRING "${mvkit_project_readme}" ${mvkit_notices_begin} -1 MVKIT_SDK_THIRD_PARTY_NOTICES)
string(REPLACE "(3rdparty/eigen/" "(third-party/eigen/"
  MVKIT_SDK_THIRD_PARTY_NOTICES "${MVKIT_SDK_THIRD_PARTY_NOTICES}")
string(REPLACE "(cmake/Dependencies.cmake)" "(Dependencies.cmake)"
  MVKIT_SDK_THIRD_PARTY_NOTICES "${MVKIT_SDK_THIRD_PARTY_NOTICES}")
if(NOT MVKIT_USE_EIGEN)
  string(REGEX REPLACE "\n[|] Eigen [|][^\n]*\n" "\n"
    MVKIT_SDK_THIRD_PARTY_NOTICES "${MVKIT_SDK_THIRD_PARTY_NOTICES}")
endif()
set(mvkit_sdk_docs "${CMAKE_CURRENT_BINARY_DIR}/generated/sdk-docs")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/docs/api.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/docs/features.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/docs/runtime.md")
file(MAKE_DIRECTORY "${mvkit_sdk_docs}/docs")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/SDKReadme.md.in"
  "${mvkit_sdk_docs}/README.md" @ONLY)
foreach(document api features runtime)
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/docs/${document}.md" mvkit_document)
  string(REPLACE "(../include/mvkit/" "(../${MVKIT_SDK_INCLUDE_RELATIVE}/mvkit/"
    mvkit_document "${mvkit_document}")
  file(WRITE "${mvkit_sdk_docs}/docs/${document}.md" "${mvkit_document}")
endforeach()
install(FILES LICENSE NOTICE "${mvkit_sdk_docs}/README.md" cmake/Dependencies.cmake
  DESTINATION "${CMAKE_INSTALL_DATADIR}/MVKit")
install(DIRECTORY "${mvkit_sdk_docs}/docs/" DESTINATION "${CMAKE_INSTALL_DATADIR}/MVKit/docs")
if(MVKIT_USE_EIGEN)
  # MPL 覆盖源码与原始声明随二进制 SDK 提供，不进入公开 include 树。
  install(DIRECTORY "${MVKIT_3RDPARTY_DIR}/eigen/"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/MVKit/third-party/eigen")
endif()

if(WIN32)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/RuntimeInstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/RuntimeInstall.cmake" @ONLY)
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/RuntimeInstall-$<CONFIG>.cmake"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/RuntimeInstall.cmake")
  install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/generated/RuntimeInstall-$<CONFIG>.cmake")
endif()

configure_package_config_file(
  cmake/templates/MVKitConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/MVKitConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MVKit
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/MVKitConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/MVKitConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/MVKitConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MVKit
)
install(EXPORT MVKitTargets
  FILE MVKitTargets.cmake
  NAMESPACE mvkit::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MVKit
)
