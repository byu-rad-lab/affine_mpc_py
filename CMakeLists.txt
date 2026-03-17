# 3.21 allows for IMPORTED_RUNTIME_ARTIFACTS
# 3.27 allows for new way to import Python/Pybind11
# 3.28 allows for EXCLUDE_FROM_ALL in FetchContent
cmake_minimum_required(VERSION 3.28)
project(affine_mpc_py LANGUAGES CXX)

# Stub files provide IDE support (signature help) for compiled bindings
option(GENERATE_STUBS "Generate stub files for compiled bindings" OFF)

# -----------------------------------------
# Dependencies
# -----------------------------------------
include(FetchContent)
include(cmake/extern_pybind11.cmake)
include(cmake/extern_ampc.cmake)

# -----------------------------------------
# Python bindings
# -----------------------------------------
pybind11_add_module(_bindings MODULE
  src/bindings/affine_mpc_py_module.cpp
  src/bindings/py_options.cpp
  src/bindings/py_parameterization.cpp
  src/bindings/py_osqp_settings.cpp
  src/bindings/py_solve_status.cpp
  src/bindings/py_mpc_base.cpp
  src/bindings/py_condensed_mpc.cpp
  src/bindings/py_sparse_mpc.cpp
  src/bindings/py_mpc_logger.cpp
)

# Set output directory to match package structure
set_target_properties(_bindings PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/affine_mpc"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/affine_mpc"
)

# Link to the affine_mpc library
target_link_libraries(_bindings PRIVATE affine_mpc::affine_mpc)

# Set runtime path for shared libraries systems to be relative to the module
if(${BUILD_SHARED_LIBS})
  if(APPLE)
      set_target_properties(_bindings PROPERTIES INSTALL_RPATH "@loader_path")
  elseif(UNIX)
      set_target_properties(_bindings PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()
endif()

# -----------------------------------------
# Stub generation
# -----------------------------------------
if(${GENERATE_STUBS})
  # Locate build environment dependencies
  set(GET_STUBGEN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/get_stubgen_path.py")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${GET_STUBGEN_PATH}"
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

  # Path to custom stub generation script
  set(GEN_STUBS "${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py")

  # Post-build command to copy generate stubs
  add_custom_command(TARGET _bindings POST_BUILD
    # need __init__.py for package recognition (and everything it references)
    COMMAND ${CMAKE_COMMAND} -E copy
      ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc/*.py
      ${CMAKE_CURRENT_BINARY_DIR}/affine_mpc/.

    # generate stubs
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${PY_PATHS}"
      "${Python3_EXECUTABLE}" "${GEN_STUBS}" "${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

# -----------------------------------------
# Installation
# -----------------------------------------

# Install the compiled bindings module
install(TARGETS _bindings
  DESTINATION affine_mpc
  COMPONENT python_bindings
)

if(BUILD_SHARED_LIBS)
  # Install shared libraries for affine_mpc and osqp dependencies
  install(IMPORTED_RUNTIME_ARTIFACTS affine_mpc osqp
    RUNTIME DESTINATION affine_mpc
    LIBRARY DESTINATION affine_mpc
    COMPONENT python_bindings
  )
endif()

if(GENERATE_STUBS)
  # Install generated stub file for the bindings module (IDE support)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/_bindings.pyi
    DESTINATION affine_mpc
    COMPONENT python_bindings
    OPTIONAL
  )
endif()

# if(GENERATE_STUBS)
#   install(
#     DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/
#     DESTINATION affine_mpc
#     COMPONENT python_bindings
#     FILES_MATCHING
#       PATTERN "*.pyi"
#       PATTERN "py.typed"
#   )
# endif()

# Install regular Python files for the package
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc
  DESTINATION .
  COMPONENT python_bindings
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.pyi"
    PATTERN "py.typed"
)
