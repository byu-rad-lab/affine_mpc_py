cmake_minimum_required(VERSION 3.14)
project(affine_mpc_py)

add_subdirectory(lib/affine_mpc_dev)

# set(PYBIND11_VERSION "2.9.2" CACHE STRING "Set desired version of Pybind11")
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
message(STATUS "Found pybind11 ${pybind11_VERSION}")

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
set_target_properties(_bindings PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/affine_mpc
  LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/affine_mpc
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/affine_mpc
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/affine_mpc
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


install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py /tmp/)")
install(FILES /tmp/stubs/affine_mpc/_bindings.pyi DESTINATION ${SKBUILD_PLATLIB_DIR}/affine_mpc)


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

# # Only build python tests if we are building the python bindings
# if(BUILD_TESTS)
#   # Add target for use with `cmake --build . --target test_bindings`
#   add_custom_target(test_bindings
#     COMMAND pytest
#     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#   )
#   # Add test for use with `ctest`
#   add_test(NAME test_bindings
#     COMMAND pytest
#     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
# endif()
