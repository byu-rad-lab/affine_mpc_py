if(WIN32)
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
endif()

set(PYBIND11_PYTHON_VERSION 3 CACHE STRING "Version of Python to use")
set(PYBIND11_VERSION 3.0.1 CACHE STRING "Version of pybind11 to use")

set(PYBIND11_FINDPYTHON ON)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 ${PYBIND11_VERSION} CONFIG QUIET)

if(NOT ${pybind11_FOUND})
  message(STATUS "System install of Pybind11 not found - using FetchContent")
  message(STATUS
    "pybind11 can be installed system-wide via `pip install pybind11[global]`"
  )

  FetchContent_Declare(pybind11_extern
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v${PYBIND11_VERSION}
  )
  FetchContent_MakeAvailable(pybind11_extern)

  message(STATUS "Pybind11 version: ${PYBIND11_VERSION}")
else()
  message(STATUS "Pybind11 version: ${pybind11_VERSION}")
  if(NOT ${pybind11_VERSION} VERSION_GREATER_EQUAL "2.4.3")
    message(STATUS "Warning: lowest tested version of Pybind11 is 2.4.3")
  endif()
endif()

