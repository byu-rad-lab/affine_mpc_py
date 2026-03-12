#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc {
namespace py = pybind11;


void moduleAddCondensedMPC(py::module& m)
{
  py::class_<CondensedMPC, MPCBase> mpc(m, "CondensedMPC",
                                        R"(
MPC formulation where only parameterization control points are optimization
design variables (condensed QP).

Eliminates state variables analytically by implicitly wrapping the model into
the const function rather than as a constraint , resulting in a smaller dense
QP. Preferred for shorter horizons and lower-dimensional problems.
Converts the MPC problem to QP form for OSQP, using input parameterization.
                                         )");

  mpc.def(
      py::init<const int, const int, const Parameterization&, const Options&>(),
      R"(
Construct CondensedMPC with specified input parameterization and MPC
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
Construct CondensedMPC with no parameterization (full input trajectory will be
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
  //       [](CondensedMPC& self, Eigen::Ref<Eigen::VectorXd> u_traj) {
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
  //       [](CondensedMPC& self) {
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
  //     [](CondensedMPC& self, Eigen::Ref<Eigen::VectorXd> x_traj) {
  //       self.getPredictedStateTrajectory(x_traj);
  //       return x_traj;
  //     },
  //     "Get predicted state trajectory from previous solve",
  //     py::arg("x_traj"));
  // mpc.def(
  //     "getPredictedStateTrajectory",
  //     [](CondensedMPC& self) {
  //       Eigen::VectorXd x_traj{self.getStateDim() * self.getHorizonSteps()};
  //       self.getPredictedStateTrajectory(x_traj);
  //       return x_traj;
  //     },
  //     "Get predicted state trajectory from previous solve");
  //
  // mpc.def("setInputLimits", &CondensedMPC::setInputLimits,
  //         "Set input saturation limits", py::arg("u_min"), py::arg("u_max"));
  // mpc.def("setStateLimits", &CondensedMPC::setStateLimits,
  //         "Set state saturation limits", py::arg("x_min"), py::arg("x_max"));
  // mpc.def("setSlewRate", &CondensedMPC::setSlewRate,
  //         "Set slew rate constraint limits", py::arg("u_slew"));
}

} // namespace affine_mpc
