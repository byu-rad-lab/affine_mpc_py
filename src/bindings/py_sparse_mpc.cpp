#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"

namespace affine_mpc {
namespace py = pybind11;


void moduleAddSparseMPC(py::module& m)
{
  py::class_<SparseMPC, MPCBase> mpc(m, "SparseMPC",
                                     R"(
MPC formulation where the predicted state trajectory and parameterization
control points are both optimization design variables (sparse QP).

Retains state and input variables with model as an optimization constraint,
exploiting sparsity for scalability. Preferred for longer horizons or larger
state dimensions. Converts the MPC problem to QP form for OSQP, using input
parameterization.
                                     )");

  mpc.def(
      py::init<const int, const int, const Parameterization&, const Options&>(),
      R"(
Construct SparseMPC with specified input parameterization and MPC
configuration options.

Args:
   state_dim: State vector dimension.
   input_dim: Input vector dimension.
   param: Input trajectory parameterization.
   opts: Optional MPC configuration features to enable.
           )",
      py::arg("state_dim"), py::arg("input_dim"), py::arg("param"),
      py::arg("opts") = Options());

  mpc.def(py::init<const int, const int, const int, const Options&>(),
          R"(
Construct SparseMPC with no parameterization (full input trajectory will be
optimized) and MPC configuration options.

Args:
   state_dim: State vector dimension.
   input_dim: Input vector dimension.
   horizon_steps: Number of discrete steps in the MPC horizon.
   opts: Optional MPC configuration features to enable.
           )",
          py::arg("state_dim"), py::arg("input_dim"), py::arg("horizon_steps"),
          py::arg("opts") = Options());

  //   mpc.def(
  //       "getInputTrajectory",
  //       [](SparseMPC& self, Eigen::Ref<Eigen::VectorXd> u_traj) {
  //         self.getInputTrajectory(u_traj);
  //         return u_traj;
  //       },
  //       R"(
  // Get optimal trajectory of inputs from previous solve evaluated at each step
  // in the prediction horizon.
  //
  // Args:
  //     u_traj (NDArray[numpy.float64]): Array in which to store the trajectory
  //     (same memory as output).
  // Return:
  //     u_traj (NDArray[numpy.float64]): Optimized input trajectory (same
  //     memory as function argument)
  //       )",
  //       py::arg("u_traj"));
  //   mpc.def(
  //       "getInputTrajectory",
  //       [](SparseMPC& self) {
  //         Eigen::VectorXd u_traj{self.getInputDim() *
  //         self.getHorizonSteps()}; self.getInputTrajectory(u_traj); return
  //         u_traj;
  //       },
  //       R"(
  // Get optimal trajectory of inputs from previous solve evaluated at each step
  // in the prediction horizon.
  //
  // Return:
  //     u_traj (NDArray[numpy.float64]): Optimized input trajectory (same
  //     memory as function argument)
  //       )");

  // mpc.def(
  //     "getPredictedStateTrajectory",
  //     [](SparseMPC& self, Eigen::Ref<Eigen::VectorXd> x_traj) {
  //       self.getPredictedStateTrajectory(x_traj);
  //       return x_traj;
  //     },
  //     "Get predicted state trajectory from previous solve",
  //     py::arg("x_traj"));
  // mpc.def(
  //     "getPredictedStateTrajectory",
  //     [](SparseMPC& self) {
  //       Eigen::VectorXd x_traj{self.getStateDim() * self.getHorizonSteps()};
  //       self.getPredictedStateTrajectory(x_traj);
  //       return x_traj;
  //     },
  //     "Get predicted state trajectory from previous solve");
  //
  // mpc.def("setInputLimits", &SparseMPC::setInputLimits,
  //         "Set input saturation limits", py::arg("u_min"), py::arg("u_max"));
  // mpc.def("setStateLimits", &SparseMPC::setStateLimits,
  //         "Set state saturation limits", py::arg("x_min"), py::arg("x_max"));
  // mpc.def("setSlewRate", &SparseMPC::setSlewRate,
  //         "Set slew rate constraint limits", py::arg("u_slew"));
}

} // namespace affine_mpc
