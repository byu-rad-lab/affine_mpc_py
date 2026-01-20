# 3.21 allows for IMPORTED_RUNTIME_ARTIFACTS
# 3.27 allows for new way to import Python/Pybind11
# 3.28 allows for EXCLUDE_FROM_ALL in FetchContent
cmake_minimum_required(VERSION 3.28)
project(affine_mpc_py LANGUAGES CXX)

# for manual builds, scikit will install and this isn't needed
option(GENERATE_STUBS "Generate stub files for compiled bindings" ON)

include(FetchContent)
include(cmake/extern_pybind11.cmake)
include(cmake/extern_ampc.cmake)

pybind11_add_module(_bindings MODULE
  src/bindings/affine_mpc_py_module.cpp
  src/bindings/py_mpc_base.cpp
  src/bindings/py_implicit_mpc.cpp
  src/bindings/py_bspline_mpc.cpp
  src/bindings/py_mpc_logger.cpp
  src/bindings/py_osqp_settings.cpp
)
set_target_properties(_bindings PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/affine_mpc"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/affine_mpc"
)
target_link_libraries(_bindings PRIVATE affine_mpc::affine_mpc)

if(${BUILD_SHARED_LIBS})
  if(APPLE)
      set_target_properties(_bindings PROPERTIES INSTALL_RPATH "@loader_path")
  elseif(UNIX)
      set_target_properties(_bindings PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()
endif()

install(TARGETS _bindings
  DESTINATION affine_mpc
  COMPONENT python_bindings
)

if(BUILD_SHARED_LIBS)
  install(IMPORTED_RUNTIME_ARTIFACTS affine_mpc osqp
    RUNTIME DESTINATION affine_mpc
    LIBRARY DESTINATION affine_mpc
    COMPONENT python_bindings
  )
endif()

if(${GENERATE_STUBS})

  set(GEN_STUBS "${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py")
  set(STUBGEN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/get_stubgen_path.py")

  # Locate build environment dependencies
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${STUBGEN_PATH}"
    OUTPUT_VARIABLE BUILD_ENV_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pystub_found
  )

  # Ensure build environment dependencies were found
  if(NOT _pystub_found EQUAL 0)
    message(WARNING "Could not locate pybind11-stubgen in build environment."
      " Stub generation may fail."
    )
  endif()

  # Allow PYTHONPATH to find both build environment and built module
  if(WIN32)
    set(PYPATH_SEPARATOR ";")
  else()
    set(PYPATH_SEPARATOR ":")
  endif()
  set(PY_PATHS
    "${BUILD_ENV_PATH}${PYPATH_SEPARATOR}${CMAKE_CURRENT_BINARY_DIR}"
  )

  add_custom_command(TARGET _bindings POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
      ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc/*.py
      ${CMAKE_CURRENT_BINARY_DIR}/affine_mpc/.

    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${PY_PATHS}"
      "${Python3_EXECUTABLE}" "${GEN_STUBS}" "${CMAKE_CURRENT_BINARY_DIR}"
  )

  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/_bindings.pyi
    DESTINATION affine_mpc
    COMPONENT python_bindings
    OPTIONAL
  )
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc
  DESTINATION .
  COMPONENT python_bindings
  FILES_MATCHING PATTERN "*.py"
)
