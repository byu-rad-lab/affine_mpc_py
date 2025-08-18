cmake_minimum_required(VERSION 3.15)
project(affine_mpc_py LANGUAGES CXX C)
set(CMAKE_POLICY_VERSION_MINIMUM 3.15)

set(CMAKE_BUILD_TYPE Release)

option(GENERATE_STUBS "Whether or not to generate stub files for compiled bindings")

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
FetchContent_Declare(
  ampc_ext
  # TODO: change git info when it becomes public
  GIT_REPOSITORY github:byu-rad-lab/affine_mpc_dev
  GIT_TAG        origin/no_python
)
FetchContent_MakeAvailable(ampc_ext)

pybind11_add_module(_bindings MODULE
  src/bindings/affine_mpc_py_module.cpp
  src/bindings/py_mpc_base.cpp
  src/bindings/py_implicit_mpc.cpp
  src/bindings/py_bspline_mpc.cpp
  src/bindings/py_mpc_logger.cpp
  src/bindings/py_osqp_settings.cpp
)
set_target_properties(_bindings PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/affine_mpc
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/affine_mpc
)

###### static library version
target_link_libraries(_bindings PUBLIC affine_mpc_static)
install(TARGETS _bindings
  LIBRARY DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc
  RUNTIME DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc
)

###### shared library version
# target_link_libraries(_bindings PUBLIC affine_mpc)
# set_target_properties(_bindings PROPERTIES
#   INSTALL_RPATH "$ORIGIN"
#   BUILD_WITH_INSTALL_RPATH TRUE
# )
# install(TARGETS _bindings osqp affine_mpc
#   LIBRARY DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc
#   RUNTIME DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc
# )

if("${Python3_EXECUTABLE}" STREQUAL "")
  # set(Python3_EXECUTABLE "python")
  set(Python3_EXECUTABLE "$ENV{VIRTUAL_ENV}/bin/python")
endif()

if(${GENERATE_STUBS})
  add_custom_command(TARGET _bindings POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Running post-build script for _bindings..."
    COMMAND ${CMAKE_COMMAND} -E echo "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}"
    COMMAND ${CMAKE_COMMAND} -E echo "venv=$ENV{VIRTUAL_ENV}"
    COMMAND ${CMAKE_COMMAND} -E echo "Python3_EXECUTABLE=${Python3_EXECUTABLE}"


    # COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${SKBUILD_PLATLIB_DIR}
    # COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc/* ${CMAKE_CURRENT_BINARY_DIR}/affine_mpc/.
    # COMMAND PYTHONPATH="${SKBUILD_PLATLIB_DIR}" ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py
    # COMMAND PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}" python ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}" ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${CMAKE_CURRENT_BINARY_DIR}
    # COMMAND ls -l ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc
    # COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/_bindings.pyi ${SKBUILD_PLATLIB_DIR}/affine_mpc/.
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/_bindings.pyi ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc/.
    # COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/_bindings.pyi /tmp/.
    COMMAND ${CMAKE_COMMAND} -E echo "Post-build script complete."
  )
endif()

# TODO: figure out how to automatically build stub file and package it.
# So far scikit-build-core's temporary build env cannot find everything it needs to.
# It seemed like I was able to manually specify dependencies, but I can't get the stub
# file to auto-generate into the TMP folder

# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc/* DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/affine_mpc/.)
# install(CODE "execute_process(COMMAND PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}" pybind11-stubgen -o ${TMP}/stubs affine_mpc --numpy-array-use-type-var)")
# install(CODE "execute_process(COMMAND pybind11-stubgen -o ${CMAKE_CURRENT_BINARY_DIR}/stubs affine_mpc --numpy-array-use-type-var)")
# install(CODE "execute_process(COMMAND ls ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc)")

# set(STUB_DIR "${TMP}/stubs")
# set(STUB_DIR "${SKBUILD_PLATLIB_DIR}/stubs")

# install(CODE "execute_process(COMMAND PYTHONPATH=${SKBUILD_PLATLIB_DIR} pybind11-stubgen -o ${STUB_DIR} affine_mpc --numpy-array-use-type-var)")
# install(CODE "execute_process(COMMAND ls ${TMP}/stubs/affine_mpc)")
# install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/reformat_stubs.py)")
# install(CODE "execute_process(COMMAND black --pyi ${STUB_DIR}/affine_mpc)")
# install(FILES ${STUB_DIR}/affine_mpc/_bindings.pyi DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc)

# install(CODE "execute_process(COMMAND PYTHONPATH=${SKBUILD_PLATLIB_DIR} ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py)")
# install(CODE "execute_process(COMMAND PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR})")
# install(CODE "execute_process(COMMAND echo $ENV{PYTHONPATH} > /tmp/echo_output.txt)")
# install(CODE "execute_process(COMMAND ls ${SKBUILD_PLATLIB_DIR} > /tmp/ls_output.txt)")

# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc/* DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/affine_mpc
  DESTINATION ${SKBUILD_PLATLIB_DIR}
  FILES_MATCHING PATTERN "*.py"
)
install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${SKBUILD_PLATLIB_DIR})")
install(FILES ${STUB_DIR}/affine_mpc/_bindings.pyi DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc)

# install(FILES ${TMP}/stubs/affine_mpc/_bindings.pyi DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc)
