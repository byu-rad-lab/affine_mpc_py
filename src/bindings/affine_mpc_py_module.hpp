#ifndef AFFINE_MPC_MODULE_HPP
#define AFFINE_MPC_MODULE_HPP

#include <pybind11/pybind11.h>

namespace affine_mpc {
namespace py = pybind11;

void moduleAddOptions(py::module& m);
void moduleAddParameterization(py::module& m);
void moduleAddOsqpSettings(py::module& m);
void moduleAddMPCBase(py::module& m);
void moduleAddCondensedMPC(py::module& m);
void moduleAddSparseMPC(py::module& m);
void moduleAddMpcLogger(py::module& m);

} // namespace affine_mpc

#endif // AFFINE_MPC_MODULE_HPP
