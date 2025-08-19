cmake_minimum_required(VERSION 3.15)
project(affine_mpc_py LANGUAGES CXX C)
set(CMAKE_POLICY_VERSION_MINIMUM 3.15)

set(CMAKE_BUILD_TYPE Release)

option(GENERATE_STUBS "Whether or not to generate stub files for compiled bindings")
option(USE_SHARED_LIBRARIES "Whether or not to compile shared vs static libraries")

if("${SKBUILD_PLATLIB_DIR}" STREQUAL "")
  set(OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
else()
  set(OUTPUT_DIR "${SKBUILD_PLATLIB_DIR}")
endif()

if(WIN32)
  string(REPLACE "\\" "/" TMP $ENV{TEMP})
  
  # help Windows find pybind11 installation from venv
  if("${pybind11_DIR}" STREQUAL "" OR "${pybind11_DIR}" STREQUAL "pybind11_DIR-NOTFOUND")
    string(REPLACE "\\" "/" PYDIR "$ENV{VIRTUAL_ENV}/share/cmake/pybind11")
    if(EXISTS "${PYDIR}")
      set(pybind11_DIR "${PYDIR}")
      message(STATUS "pybind11_DIR: ${pybind11_DIR}")
    else()
      message(STATUS "pybind11_DIR not found in venv - try `pip install pybind11[global]`")
    endif()
  endif()
else()
  set(TMP "/tmp")
endif()

set(PYBIND11_FINDPYTHON ON)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 3.0.0 CONFIG REQUIRED)
message(STATUS "Found pybind11: ${pybind11_VERSION}")

include(FetchContent)
include(cmake/extern_ampc.cmake)
include(cmake/extern_osqp.cmake)

pybind11_add_module(_bindings MODULE
  src/bindings/affine_mpc_py_module.cpp
  src/bindings/py_mpc_base.cpp
  src/bindings/py_implicit_mpc.cpp
  src/bindings/py_bspline_mpc.cpp
  src/bindings/py_mpc_logger.cpp
  src/bindings/py_osqp_settings.cpp
)
set_target_properties(_bindings PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/affine_mpc
  RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}/affine_mpc
)
if(${USE_SHARED_LIBRARIES})
  target_link_libraries(_bindings PUBLIC affine_mpc)
  set_target_properties(_bindings PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  install(TARGETS _bindings osqp affine_mpc
    LIBRARY DESTINATION ${OUTPUT_DIR}/affine_mpc
    RUNTIME DESTINATION ${OUTPUT_DIR}/affine_mpc
  )
else()
  target_link_libraries(_bindings PUBLIC affine_mpc_static)
  install(TARGETS _bindings
    LIBRARY DESTINATION ${OUTPUT_DIR}/affine_mpc
    RUNTIME DESTINATION ${OUTPUT_DIR}/affine_mpc
  )
endif()

if("${Python3_EXECUTABLE}" STREQUAL "")
  set(Python3_EXECUTABLE "$ENV{VIRTUAL_ENV}/bin/python")
endif()

set(STUB_DIR "${TMP}/stubs")

if(${GENERATE_STUBS})
  add_custom_command(TARGET _bindings POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Running post-build script for _bindings..."

    # copy affine_mpc python files to OUTPUT_DIR so the package can imported
    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc/* ${CMAKE_CURRENT_BINARY_DIR}/affine_mpc/.

    # run custom stubgen, which imports build version of package and saves stubs to STUB_DIR
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${CMAKE_CURRENT_BINARY_DIR}

    # copy custom stub file to package src (only if committing stub file to git)
    # COMMAND cp ${STUB_DIR}/affine_mpc/_bindings.pyi ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc/.

    COMMAND ${CMAKE_COMMAND} -E echo "Post-build script complete."
  )
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc
  DESTINATION ${OUTPUT_DIR}
  FILES_MATCHING PATTERN "*.py"
)
install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${OUTPUT_DIR})")
install(FILES ${STUB_DIR}/affine_mpc/_bindings.pyi DESTINATION ${OUTPUT_DIR}/affine_mpc)
