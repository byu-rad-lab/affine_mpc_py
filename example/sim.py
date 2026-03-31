import tempfile
import os
from pathlib import Path
from time import perf_counter

import numpy as np
import affine_mpc as ampc

import plot_sim


def main():
    tmp = Path(tempfile.gettempdir())
    save_dir = tmp / "ampc_example"
    if (save_dir / "log.npz").exists():
        answer = input("Log file already exists. Overwrite? [Y/n]: ")
        if len(answer) == 0:
            answer = "y"
        if not answer.lower().startswith("y"):
            print("Exiting.")
            return

    msd_mpc = ampc.CondensedMPC(
        state_dim=2,
        input_dim=1,
        param=ampc.Parameterization.linearInterp(
            horizon_steps=10, num_control_points=3
        ),
        opts=ampc.Options(use_input_cost=True),
    )

    A = np.array([[0, 1], [-0.6, -0.1]])
    B = np.array([0, 0.2])
    w = np.zeros(2)
    ts = 0.1
    msd_mpc.setModelContinuous2Discrete(A, B, w, ts)

    u_min = np.zeros(1)
    u_max = np.ones(1) * 3
    msd_mpc.setInputLimits(u_min, u_max)

    # slew = np.ones(1)
    # msd_mpc.setSlewRate(slew)

    Q_diag = np.array([1, 0.1])
    R_diag = np.array([0.00001])
    msd_mpc.setWeights(Q_diag, R_diag)

    x_goal = np.array([1.0, 0])
    u_goal = np.array([0.0])
    msd_mpc.setReferenceState(x_goal)
    msd_mpc.setReferenceInput(u_goal)

    msd_mpc.initializeSolver()

    logger = ampc.MPCLogger(msd_mpc, save_dir, ts)

    xk = np.zeros(2)
    t = 0.0
    tf = 15.0
    while t < tf:
        # MPC control
        start = perf_counter()
        solved = msd_mpc.solve(xk)
        if solved != ampc.SolveStatus.Success:
            print("Did not solve :(")
        uk = msd_mpc.getNextInput()
        elapsed = perf_counter() - start
        logger.logStep(t, xk, elapsed)

        # Simulate system
        xk = msd_mpc.propagateModel(xk, uk)
        t += ts

    print("Log file written to", save_dir / "log.npz")


if __name__ == "__main__":
    main()
    plot_sim.main()
