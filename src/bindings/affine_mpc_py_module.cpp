#include "affine_mpc_py_module.hpp"

// #include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace affine_mpc_py {

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Affine MPC module";

  moduleAddOptions(m);
  moduleAddParameterization(m);
  moduleAddOsqpSettings(m);
  moduleAddSolveStatus(m);
  moduleAddMPCBase(m);
  moduleAddCondensedMPC(m);
  moduleAddSparseMPC(m);
  moduleAddMpcLogger(m);
}

} // namespace affine_mpc_py
