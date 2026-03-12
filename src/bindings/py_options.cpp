#include "affine_mpc_py_module.hpp"

#include "affine_mpc/options.hpp"

namespace affine_mpc {

void moduleAddOptions(py::module& m)
{
  py::class_<Options> opt(m, "Options", R"(
Controls which optional features are enabled at MPC construction time.

All fields default to false. These options are immutable after constructing an
MPC instance.

Attributes:
    use_input_cost: Enables input regularization term in the cost function
        ((uref_k - u_k)^T R (uref_k - u_k)).
    slew_initial_input: Slew-rate constraint on initial input
        (|u0 - u_prev| <= u0_slew).
    slew_control_points: Enables slew-rate constraints on parameterization
        control points (|v_{i+1} - v_i| <= u_slew).
    saturate_states: Enables state saturation constraints.
    saturate_input_trajectory: Enables saturation of each input in the
        trajectory rather than just the control points. Only applicable for
        parameterizations with degree > 1. This adds constraints to the
        optimiztion, but can allow control points to be outside of input limits
        while keeping inputs within limits.
                          )");

  opt.def(py::init<>(), "Constructor");
  opt.def_readwrite("use_input_cost", &Options::use_input_cost);
  opt.def_readwrite("slew_initial_input", &Options::slew_initial_input);
  opt.def_readwrite("slew_control_points", &Options::slew_control_points);
  opt.def_readwrite("saturate_states", &Options::saturate_states);
  opt.def_readwrite("saturate_input_trajectory",
                    &Options::saturate_input_trajectory);
}

} // namespace affine_mpc
