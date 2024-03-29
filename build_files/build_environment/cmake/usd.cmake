# SPDX-License-Identifier: GPL-2.0-or-later

set(USD_EXTRA_ARGS
  -DBoost_COMPILER:STRING=${BOOST_COMPILER_STRING}
  -DBoost_USE_MULTITHREADED=ON
  -DBoost_USE_STATIC_LIBS=ON
  -DBoost_USE_STATIC_RUNTIME=OFF
  -DBOOST_ROOT=${LIBDIR}/boost
  -DBoost_NO_SYSTEM_PATHS=ON
  -DBoost_NO_BOOST_CMAKE=ON
  -DTBB_INCLUDE_DIRS=${LIBDIR}/tbb/include
  -DTBB_LIBRARIES=${LIBDIR}/tbb/lib/${LIBPREFIX}${TBB_LIBRARY}${LIBEXT}
  -DTbb_TBB_LIBRARY=${LIBDIR}/tbb/lib/${LIBPREFIX}${TBB_LIBRARY}${LIBEXT}
  # USD wants the tbb debug lib set even when you are doing a release build
  # Otherwise it will error out during the cmake configure phase.
  -DTBB_LIBRARIES_DEBUG=${LIBDIR}/tbb/lib/${LIBPREFIX}${TBB_LIBRARY}${LIBEXT}

  # This is a preventative measure that avoids possible conflicts when add-ons
  # try to load another USD library into the same process space.
  -DPXR_SET_INTERNAL_NAMESPACE=usdBlender

  -DPXR_ENABLE_PYTHON_SUPPORT=OFF
  -DPXR_BUILD_IMAGING=OFF
  -DPXR_BUILD_TESTS=OFF
  -DBUILD_SHARED_LIBS=OFF
  -DPYTHON_EXECUTABLE=${PYTHON_BINARY}
  -DPXR_BUILD_MONOLITHIC=ON

  # The PXR_BUILD_USD_TOOLS argument is patched-in by usd.diff. An upstream pull request
  # can be found at https://github.com/PixarAnimationStudios/USD/pull/1048.
  -DPXR_BUILD_USD_TOOLS=OFF

  -DCMAKE_DEBUG_POSTFIX=_d
  # USD is hellbound on making a shared lib, unless you point this variable to a valid cmake file
  # doesn't have to make sense, but as long as it points somewhere valid it will skip the shared lib.
  -DPXR_MONOLITHIC_IMPORT=${BUILD_DIR}/usd/src/external_usd/cmake/defaults/Version.cmake
)

ExternalProject_Add(external_usd
  URL file://${PACKAGE_DIR}/${USD_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${USD_HASH_TYPE}=${USD_HASH}
  PREFIX ${BUILD_DIR}/usd
  PATCH_COMMAND ${PATCH_CMD} -p 1 -d ${BUILD_DIR}/usd/src/external_usd < ${PATCH_DIR}/usd.diff
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${LIBDIR}/usd -Wno-dev ${DEFAULT_CMAKE_FLAGS} ${USD_EXTRA_ARGS}
  INSTALL_DIR ${LIBDIR}/usd
)

add_dependencies(
  external_usd
  external_tbb
  external_boost
)

# Since USD 21.11 the libraries are prefixed with "usd_", i.e. "libusd_m.a" became "libusd_usd_m.a".
# See https://github.com/PixarAnimationStudios/USD/blob/release/CHANGELOG.md#2111---2021-11-01
if (USD_VERSION VERSION_LESS 21.11)
  set(PXR_LIB_PREFIX "")
else()
  set(PXR_LIB_PREFIX "usd_")
endif()

if(WIN32)
  # USD currently demands python be available at build time
  # and then proceeds not to use it, but still checks that the
  # version of the interpreter it is not going to use is atleast 2.7
  # so we need this dep currently since there is no system python
  # on windows.
  add_dependencies(
    external_usd
    external_python
  )
  if(BUILD_MODE STREQUAL Release)
    ExternalProject_Add_Step(external_usd after_install
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBDIR}/usd/ ${HARVEST_TARGET}/usd
      COMMAND ${CMAKE_COMMAND} -E copy ${BUILD_DIR}/usd/src/external_usd-build/pxr/Release/${PXR_LIB_PREFIX}usd_m.lib ${HARVEST_TARGET}/usd/lib/lib${PXR_LIB_PREFIX}usd_m.lib
      DEPENDEES install
    )
  endif()
  if(BUILD_MODE STREQUAL Debug)
    ExternalProject_Add_Step(external_usd after_install
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBDIR}/usd/lib ${HARVEST_TARGET}/usd/lib
      COMMAND ${CMAKE_COMMAND} -E copy ${BUILD_DIR}/usd/src/external_usd-build/pxr/Debug/${PXR_LIB_PREFIX}usd_m_d.lib ${HARVEST_TARGET}/usd/lib/lib${PXR_LIB_PREFIX}usd_m_d.lib
      DEPENDEES install
    )
  endif()
else()
  # USD has two build options. The default build creates lots of small libraries,
  # whereas the 'monolithic' build produces only a single library. The latter
  # makes linking simpler, so that's what we use in Blender. However, running
  # 'make install' in the USD sources doesn't install the static library in that
  # case (only the shared library). As a result, we need to grab the `libusd_m.a`
  # file from the build directory instead of from the install directory.
  ExternalProject_Add_Step(external_usd after_install
    COMMAND ${CMAKE_COMMAND} -E copy ${BUILD_DIR}/usd/src/external_usd-build/pxr/lib${PXR_LIB_PREFIX}usd_m.a ${HARVEST_TARGET}/usd/lib/lib${PXR_LIB_PREFIX}usd_m.a
    DEPENDEES install
  )
endif()
