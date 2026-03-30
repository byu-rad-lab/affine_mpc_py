import numpy as np
import os
import tempfile
from pathlib import Path

import affine_mpc as ampc


def test_mpc_logger_interface():
    try:
        tmp = Path(tempfile.gettempdir())
        folder = tmp / "test_bindings"
        paramfile = "params.yaml"

        mpc = ampc.CondensedMPC(2, 1, ampc.Parameterization.linearInterp(10, 3))

        A = np.array([[0, 1], [-0.6, -0.1]])
        B = np.array([0, 0.2])
        w = np.zeros(2)
        ts = 0.1
        mpc.setModelContinuous2Discrete(A, B, w, ts)
        u_min = np.zeros(1)
        u_max = np.ones(1) * 3
        mpc.setInputLimits(u_min, u_max)
        Q_diag = np.array([1, 0.1])
        mpc.setStateWeights(Q_diag)
        x_goal = np.array([1.0, 0])
        mpc.setReferenceState(x_goal)
        mpc.initializeSolver()

        logger = ampc.MPCLogger(
            mpc=mpc,
            save_dir=folder,
            ts=ts,
            prediction_stride=1,
            log_control_points=False,
            save_name="log",
        )
        print("made logger")

        xk = np.zeros(2)
        t = 0.0
        tf = 1.0
        while t < tf:
            # MPC control
            solved = mpc.solve(xk)
            if solved != ampc.SolveStatus.Success:
                print("Did not solve.")
            else:
                print("solved")
            uk = mpc.getNextInput()
            logger.logStep(t=t, x0=xk, solve_time=-1.0)
            print("logged step")

            # Simulate system
            xk = mpc.propagateModel(xk, uk)
            t += ts

        print("made logger")
        logger.logStep(t, xk)
        print("logged step")
        logger.writeParamFile(filename="params.yaml")
        print("wrote params")
        assert hasattr(logger, "finalize")

        del logger

        assert (folder / paramfile).exists()
        assert (folder / "log.npz").exists()
        # os.remove(folder)

    except:
        assert False


if __name__ == "__main__":
    test_mpc_logger_interface()
    print("All tests passed!")
