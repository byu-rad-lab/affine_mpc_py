#include "affine_mpc_py_module.hpp"

#include "affine_mpc/parameterization.hpp"

namespace affine_mpc {

void moduleAddParameterization(py::module& m)
{
  py::class_<Parameterization> opt(m, "Parameterization", R"(
Encapsulates B-spline input trajectory parameterization for MPC.

Provides constructors and factory methods for generating knot vectors and
metadata to map control points to per-step input trajectories over the
prediction horizon.

The preferred usage is via the static factory methods for common
parameterizations.

Attributes:
    knots: Full knot vector of size num_control_points + degree + 1.
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of B-spline control points.
    degree: Degree of B-spline polynomials. Must be less than num_control_points.

Methods:
    validateKnots: Validates the knot vector for correctness.

Static Methods:
    moveBlocking: Factory method for move-blocking parameterization.
    linearInterp: Factory method for linear interpolation parameterization.
    bspline: Factory method for clamped B-spline parameterization.
                          )");

  opt.def(py::init<const int, const int, const int>(), R"(
Direct constructor for uniform clamped B-spline parameterization.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of B-spline control points.
    degree: Degree of B-spline polynomials. Must be less than num_control_points.
          )",
          py::arg("horizon_steps"), py::arg("num_control_points"),
          py::arg("degree"));

  opt.def(py::init<const int, const int, const int,
                   const Eigen::Ref<const Eigen::VectorXd>&>(),
          R"(
Direct constructor for advanced use cases with custom knot vector
(e.g. unclamped B-splines).

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of B-spline control points.
    degree: Degree of B-spline polynomials. Must be less than num_control_points.
    knots: Full knot vector of size num_control_points + degree + 1. Must be
        non-decreasing. First knot must be 0 and last knot must be horizon_steps-1.
          )",
          py::arg("horizon_steps"), py::arg("num_control_points"),
          py::arg("degree"), py::arg("knots"));

  opt.def(
      "validateKnots",
      [](const Parameterization& self) {
        std::string error_msg;
        const bool valid{self.validateKnots(error_msg)};
        return std::make_tuple(valid, error_msg);
      },
      R"(
Validates the knot vector for correctness.

Returns:
    valid: True if the knot vector is valid, false otherwise.
    error_msg: If valid is false, this will contain a message describing the
        issue with the knot vector.
          )");

  opt.def_readwrite("knots", &Parameterization::knots);
  opt.def_readonly("horizon_steps", &Parameterization::horizon_steps);
  opt.def_readonly("num_control_points", &Parameterization::num_control_points);
  opt.def_readonly("degree", &Parameterization::degree);

  opt.def_static(
      "moveBlocking",
      py::overload_cast<const int, const int>(&Parameterization::moveBlocking),
      R"(
Factory method for uniform move-blocking parameterization.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of B-spline control points (i.e. number of change
        points in the input trajectory.

returns:
    out: Parameterization instance with uniform move-blocking structure.
)",
      py::arg("horizon_steps"), py::arg("num_control_points"));

  opt.def_static(
      "moveBlocking",
      py::overload_cast<const int, const Eigen::Ref<const Eigen::VectorXd>&>(
          &Parameterization::moveBlocking),
      R"(
Factory method for move-blocking parameterization with custom change points.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    change_points: Vector of change point locations.

returns:
    out: Parameterization instance with move-blocking structure.
)",
      py::arg("horizon_steps"), py::arg("change_points"));

  opt.def_static(
      "linearInterp",
      py::overload_cast<const int, const int>(&Parameterization::linearInterp),
      R"(
Factory method for uniform linear interpolation parameterization.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of control points.

returns:
    out: Parameterization instance with uniform linear interpolation structure.
)",
      py::arg("horizon_steps"), py::arg("num_control_points"));

  opt.def_static(
      "linearInterp",
      py::overload_cast<const int, const Eigen::Ref<const Eigen::VectorXd>&>(
          &Parameterization::linearInterp),
      R"(
Factory method for linear interpolation parameterization with custom endpoints.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    endpoints: Vector of linear segment endpoint locations. Must include 0 and
        horizon_steps - 1.

returns:
    out: Parameterization instance with linear interpolation structure.
)",
      py::arg("horizon_steps"), py::arg("change_points"));

  opt.def_static("bspline",
                 py::overload_cast<const int, const int, const int>(
                     &Parameterization::bspline),
                 R"(
Factory method for uniform clamped B-spline parameterization.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of control points.
    degree: B-spline polynomial degree.

returns:
    out: Parameterization instance with uniform clamped B-spline structure.
)",
                 py::arg("horizon_steps"), py::arg("num_control_points"),
                 py::arg("degree"));

  opt.def_static("bspline",
                 py::overload_cast<const int, const int, const int,
                                   const Eigen::Ref<const Eigen::VectorXd>&>(
                     &Parameterization::bspline),
                 R"(
Factory method for clamped B-spline parameterization with custom active knots.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of control points.
    degree: B-spline polynomial degree.
    active_knots: Vector of active knot locations.

returns:
    out: Parameterization instance with clamped B-spline structure.
)",
                 py::arg("horizon_steps"), py::arg("num_control_points"),
                 py::arg("degree"), py::arg("active_knots"));
}

} // namespace affine_mpc
