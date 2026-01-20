# affine_mpc

## Overview

This repository contains Python bindings for the `affine_mpc` C++ library, which provides a convenient interface to the OSQP solver library in order to solve Model Predictive Control (MPC) problems that use a discrete-time affine time-invariant model.

## License

This work is licensed with the BSD 3-clause license, see `LICENSE` file for details.

## Dependencies

Note that this project was developed using both Ubuntu 20.04 and 22.04 with the GCC
compiler (version 11).

#### Required:

- C++ Compiler (GCC, Clang, MSVC)
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) (>3.4)
  - Can install from [source](https://gitlab.com/libeigen/eigen)
- [OSQP](https://osqp.org/docs/get_started/) (0.6.x, not yet 1.x)
  - This project will locally clone and build OSQP for you if not on your system
- [Pybind11](https://pybind11.readthedocs.io/en/stable/index.html) (>2.4)
  - Will be automatically installed to build environment (and deleted after) when you run `pip install .` (recommended)
  - Can be installed to system or venv if you want to specify version
    - Via pip: `pip install "pybind11[global]"` (recommended system-wide install)
    - Via apt (Ubuntu): `sudo apt install pybind11-dev`
    - Via pacman (Arch): `sudo pacman -S pybind11`
    - Other systems: search for pybing11 in your package manager
- [NumPy](https://numpy.org/) (just needs to be compatible with pybind11 version - usually latest is fine)
  - Will be automatically installed to venv when you run `pip install .`

## Building from Source

First, clone the repository and cd into the top-level directory.

### With pip (recommended)

```sh
pip install .
```

#### Optionally use shared libraries

**Might not work on Windows.**

[More info on RPATH.](https://scikit-build-core.readthedocs.io/en/latest/guide/dynamic_link.html)

Shared libraries must be found at runtime.
This project is configured to set the runtime path (RPATH) of the python bindings to it's parent directory (meaning it
will look for the shared libraries in the same directory as the python bindings - in the installed `affine_mpc` folder);
however, Windows does not support RPATH.
To use this on Windows, you would most likely need to use `os.add_dll_directory(<path>)` in your Python code (maybe in `__init__.py` of the package?).
It may also not work if trying to use system-installed shared libraries for Linux/MacOS (I think it would, but it has not been tested).

```sh
pip install . --config-settings=cmake.define.BUILD_SHARED_LIBS=ON
```

### Manually with CMake (for development - not recommended for installation)

You will need some Python packages first (numpy, pybind11, pybind11-stubgen, black):

```sh
pip install -r build_requirements.txt
```

#### If you have the Ninja generator installed

```sh
mkdir build install
cmake -S . -B build -G "Ninja"
cmake --build build --config Release
cmake --install build --prefix install --component python_bindings
```

#### If you do not have Ninja

```sh
mkdir build install
cmake -S . -B build
cmake --build build --config Release --parallel
cmake --install build --prefix install --component python_bindings
```

The `affine_mpc` package will now be inside of `install`.
You could set the install location to be wherever you want, but if you want it to be in your Python environment then
build with pip as shown above.
To use the package, you either need to be inside of the `install` directory, add the path to your `PYTHONPATH`
environment variable, or copy the `affine_mpc` folder inside of the install directory to somewhere that is on your
Python path.
The last option is not recommended as you generally should not manually edit Python venvs.
Editing `PYTHONPATH` is not recommended either, but it works if you do not want to install with pip.
Honestly, the best option is to just install with pip.

### Verify Installation

Then in a python file you can import the package:

```python
import affine_mpc as ampc
```

## MPC Problem

The following equations show the supported cost function and constraints within the `affine_mpc` library (the underlined portions with a red label are optional):

```math
\begin{equation}
% J = \sum_{k=1}^{T+1} ||x_k - x_{k,des}||_Q + \underbrace{\sum_{k=0}^{p} ||u_k - u_{k,des}||_R}_{\textcolor{red}{\text{input cost}}}
\text{argmin} \quad || \bar{x}_T - x_T ||^2_{Q_f} + \sum_{k=1}^{T-1} || \bar{x}_k - x_k ||^2_Q + \underbrace{\sum_{i=0}^{p-1} || \bar{\nu}_i - \nu_i ||^2_R}_{\textcolor{red}{\text{input cost}}}
\end{equation}
```

```math
\begin{align}
% \text{min} &\quad \sum_{k=1}^T ||x_k - x_{k,des}||^2_Q + \underbrace{\sum_{i=0}^{p-1} ||\nu_i - \nu_{i,des}||^2_R}_{\textcolor{red}{\text{input cost}}} \\
w.r.t &\quad \nu_0,...,\nu_{p-1} \\
s.t. &\quad x_{k+1} = A x_k + B u_k + w \\
&\quad u_k = g(\nu_0,...,\nu_{p-1}) \\
&\quad u_{min} \leq u_k \leq u_{max} \\
&\quad \underbrace{x_{min} \leq x_k \leq x_{max}}_{\textcolor{red}{\text{saturate states}}} \\
&\quad \underbrace{|\nu_{i+1} - \nu_i| \leq \nu_{slew}}_{\textcolor{red}{\text{slew rate}}}
\end{align}
```

where, $`x \in \mathbb{R}^n`$ is the state, $`\bar{x} \in \mathbb{R}^n`$ is the reference state, $`u \in \mathbb{R}^m`$ is the input, $`\nu \in \mathbb{R}^m`$ is a control point used to parameterize the input trajectory, $`\bar{\nu} \in \mathbb{R}^m`$ is a reference control point, $`g`$ is the function to evaluate the parameterized input trajectory, $`T`$ is the horizon length, $`p`$ is the number of control points used to parameterize the input trajectory, the discrete-time affine model is defined by $`A \in \mathbb{R}^{n \times n}`$, $`B \in \mathbb{R}^{n \times m}`$, and $`w \in \mathbb{R}^n`$, and $`Q \in \mathbb{R}^{n \times n}`$ and $`R \in \mathbb{R}^{m \times m}`$ are positive semi-definite diagonal weighting matrices.

**NOTE:** The norm in the cost function is a weighted 2-norm where $`||x||^2_M = x^\top M x`$.

The MPC optimization problem must be converted to a QP optimization problem in order to use the OSQP solver. This [paper](https://arxiv.org/pdf/2001.04931.pdf) shows how the conversion is done. Note that Implicit MPC is what the paper calls Small Matrix Formulation.

<!--
Possible additions:
  - Terminal state weights (Q_final)
  - Terminal input weights (R_final)
  - Slew rate cost instead of constraint
  - State saturation cost?
  - Arbitrary constraint matrix addition
-->

## Examples

A Python example is available in the `example` folder. There is also a `plot_sim.py` script, which can be used to visualize the results.

## API

**Not all functionality is documented here - only the basics.** You can see the interface to all available functions by looking at the C++ header files or using iPython for some documentation of the Python bindings. A stub file for the Python bindings is included in `affine_mpc_py` to enable autocompletion in an IDE like VS Code. More on [stubgen](https://manpages.ubuntu.com/manpages/focal/man1/stubgen.1.html).

### Step 1: MPC Constructor

When you create an instance of any MPC class within the library, you must specify the number of states and inputs in your system, the horizon length and number of control points you want to use in your prediction horizon, and the options you wish to use in your cost and constraint functions (shown [above](#mpc-problem) with red labels). Note that all of the cost and constraint options default to `false`. Once you specify all of these values in the constructor, those values can not change. All of the applicable values in the cost and constraint functions can be changed, but not the size and setup of the MPC problem. All MPC classes within this library inherit from the `MPCBase` interface class (which is not usable on its own as it has no usable solver - it is used to define a consistent interface with all of the MPC classes). All MPC classes in this library have the same constructor structure as `MPCBase`:

```python
def __init__(self, num_states: int, num_inputs: int, len_horizon: int,
             num_control_points: int, use_input_cost: bool=False,
             use_slew_rate: bool=False, saturate_states: bool=False)
```

### Step 2: MPC Pre-Initialization Setup

_AFTER_ creating an MPC object with the format of the MPC problem you wish to use, **you must specify all of the applicable parameters (set the model, input saturation limits, slew rate, and state saturation limits) _BEFORE_ initializing the solver.** The state weights (Q) will default to identity while the input weights (R) will default to zero.

**Relevant Member Functions**

```python
def setModelDiscrete(Ad: NDArray, Bd: NDArray, wd: NDArray) -> None:
def setModelContinuous2Discrete(Ac: NDArray, Bc: NDArray, wc: NDArray, dt: float) -> None:
def setInputLimits(u_min: NDArray, u_max: NDArray) -> None:
def setSlewRate(u_slew: NDArray) -> None: # if slew rate enabled
def setStateLimits(x_min: NDArray, x_max: NDArray) -> None: # if state saturation enabled
```

The OSQP solver is a sparse solver and it will only keep track of elements that are non-zero at the time of initialization. This means that the model used with the solver is initialized must be non-zero wherever it is possible to have non-zero values (if you are going to be changing the model at each time step - do not worry about this if you only ever use 1 model). For example, if a Jacobian of my A matrix is a rotation matrix then I need to make sure all 9 of those elements of A are non-zero rather than passing in an identity matrix if the rotation matrix will be updated after the solver is initialized.

**If you pass in a 0 somewhere that will not be a 0 later on in the code, the solver will not track the value and the model will not be what you expect it to be.** You might get lucky, but there is no safety check or guarantee that your code will work as expected if you are not careful when you initialize the solver.

### Step 3: Initialize OSQP Solver

This library uses the OSQP solver for the optimization. After specifying the parameters from the previous section, the solver can be initialized. If you do not pass in `solver_settings` then the default settings can be found in the `OSQPSolver` constructor, which modifies a couple of OSQP's default settings.

**NOTE:** To learn more about the OSQP solver and its settings, visit the [OSQP website](https://osqp.org/docs/solver/index.html).

```python
def initializeSolver(solver_settings: OSQPSettings=None) -> bool: # returns true if successful
```

**REMINDER:** The solver utilizes sparsity, meaning that the model used when the solver is initialized needs to have the least amount of sparsity possible for your system. The solver stores the structure and values of all non-zero elements when initialized and the structure can not change. This means that if it is possible for some elements of your model to be non-zero, then they need to be non-zero when the solver is initialized.

### Step 4: Solve MPC

**You must have setup MPC and initialized the solver before you can solve!**

Once the solver is initialized, you need to specify weights and reference trajectories you wish to use:

#### Relevant Functions

**Note:** all of the following function arguents are 1D vectors/arrays.

```python
def setWeights(Q_diag: NDArray, R_diag: NDArray) -> None:
def setStateWeights(Q_diag: NDArray) -> None: # will overwrite Qf assuming Qf=Q
def setStateWeightsTerminal(Qf_diag: NDArray) -> None: # call after setting state weights
def setInputWeights(R_diag: NDArray) -> None:

def setReferenceState(x_step: NDArray) -> None:
def setReferenceStateTrajectory(x_traj: NDArray) -> None:

# if input cost enabled
def setReferenceInput(u_step: NDArray) -> None:
def setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts: NDArray) -> None:
```

#### Update parameters from pre-initialization setup

You can call any of the pre-initialization setup to update them before solving. If you want to successively linearize or affinize then you will need to update the model before each solve.

```python
setModelDiscrete(Ad, Bd, wd)
setModelContinuous2Discrete(Ac, Bc, wc, dt)
setInputLimits(u_min, u_max)
setSlewRate(u_slew) # if slew rate enabled
setStateLimits(x_min, x_max) # if state saturation enabled
```

Now you are ready to solve!

```python
def solve(x0: NDArray) -> bool: # returns true if successful
```

#### Get desired information from solve

You can get the next input to apply (the first input from the prediction horizon), the parameterized input trajectory, or the entire input trajectory over the prediction horizon. You can also get the predicted state trajectory (where MPC thinks the system will go).

Note: In Python, the following functions allow you to call them like C++ where the return value is the function argument (avoids memory allocation and copying). They can also be called with no arguments, which will allocate memory and return the array.

```python
def getNextInput([u0]) -> u0:
def getInputTrajectory([u_traj]) -> u_traj:
def getParameterizedInputTrajectory([u_traj_ctrl_pts]) -> u_traj_ctrl_pts:
def getPredictedStateTrajectory([x_traj]) -> x_traj:
```

After solving, you can update any of the parameters before solving again (if you want to change your model, the weights, reference trajectories, etc.). Then you write a loop that will continuously pass in the current state and solve for inputs.

### Step 5: Logging (Optional)

An `MPCLogger` class is provided to log data for time, predicted state trajectory, optimized input trajectory, and reference state trajectory at each time step. This can be used as a debugging tool to visualize the entire solutions generated by the solver rather than just inputs that are actually applied to the system (usually only the first input calculated in the optimization).

#### Constructor

```python
def __init__(self, mpc: MPCBase, save_location: str) -> None:
# usage
logger = MPCLogger(mpc, "$HOME/data/mpc")
```

To create a logger object you must pass in a pointer to an existing MPC object along with a string for where you want the data to be stored (default is `"/tmp/mpc_data"` and you can use `"~/"`, `"$HOME/"`, and `"${HOME}/"` at the beginning for your home directory - if the string does not start with one of these three strings or `"/"` then it is a path relative to the script):

#### Log Data

```python
def logPreviousSolve(t: float, dt: float, x0: NDArray,
                     solve_time: float=-1, write_every: int=1);
```

Data should only be logged after calling the solve function on the MPC class the logger is tracking. To perform MPC you will likely have a loop of code that solves for an input to apply at each step. The loop can also update the desired state/input trajectory, change the weights, or update other pieces of the MPC problem, but the bare bones will look something like this:

```python
while t <= t_final:
  now = time()
  # update any MPC parameters
  solved = mpc.solve(xk)
  solve_time = now - time()

  if not solved:
    # handle this how you want
  uk = mpc.getNextInput()
  logger.logPreviousSolve(t, dt, xk, solve_time)
  xk = system.propagateDynamics(xk, uk) # or publish the command somehow
  t += dt
```

The logger creates 4 txt files of data within the `save_location` directory that contain all of the data. These files can be loaded in Python with `np.loadtxt('states.txt')` and each row of data represents a single time horizon (e.g., if there were 2 states and a horizon length of 3 then a row of data would contain 6 numbers for state 1 at the first time step, state 2 at the first time step, state 1 at the second time step, etc.). This pattern follows for `inputs.txt`, `time.txt`, and `ref_states.txt`.

The `solve_time` function parameter is optional. The logger will record this time as well as the solve time reported from the solver on the same line. This is to allow the user to keep track of the time it takes to setup the MPC problem as well as the time it takes just to solve after it is setup.

The `write_every` variable is used to specify how frequently you wish to record data in a single time horizon (e.g., the default value of 1 will record all data, 2 will record every other time step, etc.).

#### Write Param File

```python
def writeParamFile(filename: str="params.yaml") -> None:
```

This function is used to write all of the parameters of the MPC problem setup to a file within the `save_location` directory. This way when you go looking back through multiple folders of data you can remember what params you used to generate plots (hopefully!). This function can be called as many times as you want, but you must specify a different file name if you want to keep multiple param files (using the same name will override the existing file). Perhaps this can be useful if you are tuning gains and want to know what they were at different points in time.

Also, the `MPCLogger` destructor is set to write a param file with the default name if you never call the function yourself. **NOTE: Python does not seem to call destructors in the correct order, so to avoid a runtime error after your script is finished you MUST have called this function manually at some point or else call `del logger` at the end of the script.**

## Testing

**Note:** The following commands are all written to be run inside of the top-level directory of the repository.

With `pytest`:

```sh
pytest
```

Without `pytest`:

```sh
python3 test/<testfile>.py
```
