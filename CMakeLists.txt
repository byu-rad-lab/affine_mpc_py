cmake_minimum_required(VERSION 3.15)
project(affine_mpc_py LANGUAGES CXX C)
set(CMAKE_POLICY_VERSION_MINIMUM 3.15)

set(CMAKE_BUILD_TYPE Release)

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
# find_package(Python 3.13 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
message(STATUS "Found pybind11: ${pybind11_VERSION}")

# add_subdirectory(lib/affine_mpc_dev)
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

install(CODE "execute_process(COMMAND pybind11_stubgen -o ${TMP} affine_mpc)")
install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/reformat_stubs.py)")
install(CODE "execute_process(COMMAND black --pyi ${TMP}/stubs/affine_mpc)")
# install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${TMP})")
install(FILES ${TMP}/stubs/affine_mpc/_bindings.pyi DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc)


# add_custom_command(TARGET _bindings POST_BUILD
#   COMMAND ${CMAKE_COMMAND} -E echo "Running post-build script for _bindings..."
#   COMMAND ${CMAKE_COMMAND} -E echo "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}"
#   COMMAND ${CMAKE_COMMAND} -E echo "venv=$ENV{VIRTUAL_ENV}"
#   # COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${SKBUILD_PLATLIB_DIR}
#   COMMAND PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}" python
#   ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py /tmp
#   # COMMAND PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}" python ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py ${CMAKE_CURRENT_BINARY_DIR}
#   # COMMAND ls -l ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc
#   # COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/_bindings.pyi ${SKBUILD_PLATLIB_DIR}/affine_mpc/.
#   # COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/_bindings.pyi /tmp/.
#   COMMAND ${CMAKE_COMMAND} -E echo "Post-build script complete."
# )
