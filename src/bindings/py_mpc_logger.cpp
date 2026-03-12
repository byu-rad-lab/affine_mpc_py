#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/mpc_logger.hpp"

namespace affine_mpc {
namespace py = pybind11;

void moduleAddMpcLogger(py::module& m)
{
  py::class_<MPCLogger> log(m, "MPCLogger", R"(
Utility for logging MPC solve results, trajectories, and parameters to files.

Logs state, input, reference trajectories, solve times, and parameterization
metadata for later analysis and visualization. Intended for use with MPCBase
and derived classes.
                            )");

  log.def(py::init<const MPCBase* const, const std::string&>(), "Constructor",
          py::arg("mpc"), py::arg("save_location") = "/tmp/mpc_data");
  log.def("logPreviousSolve", &MPCLogger::logPreviousSolve,
          R"(
Log the results of the previous MPC solve as a single row of data in a text
file. The data of the entire horizon is logged, not just the first input and
state.

Args:
    t0: Initial time of the solve (e.g. current time at solve).
    ts: Discretization time step.
    x0: Initial state at the start of the horizon (e.g. current state at solve).
    solve_time: Time taken to solve (optional). This is if you recorded the time
        separately (likely to include setup time). The solve time reported by
        OSQP is also logged separately.
    write_every: Write frequency during horizon. For example, if 2, then data
        will be written for every other time step in the horizon. However, the
        final time step will always be written regardless of this parameter.
          )",
          py::arg("t0"), py::arg("ts"), py::arg("x0"),
          py::arg("solve_time") = -1, py::arg("write_every") = 1);

  log.def("writeParamFile", &MPCLogger::writeParamFile,
          R"(
Write MPC parameters and metadata to a YAML file.

Args:
    filename: Output filename for the YAML file. Should end with .yaml or .yml.
          )",
          py::arg("filename") = "params.yaml");
}

} // namespace affine_mpc
