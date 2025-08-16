from os.path import join as pjoin
import tempfile

import abracatabra as tabby
import numpy as np
import yaml


def main(load_dir: str|None = None):
    if load_dir is None:
        tmp = tempfile.gettempdir()
        load_dir = pjoin(tmp, "ampc", "example")

    with open(pjoin(load_dir, "params.yaml"), "r") as f:
        params = yaml.safe_load(f)
        n,m = params["n"], params["m"]
        T,mu,p = params["T"], params["mu"], params["p"]
    time = np.loadtxt(pjoin(load_dir, "time.txt"))
    t_hist = time[:, 0]
    N = len(t_hist)

    states = np.loadtxt(pjoin(load_dir, "states.txt")).reshape((N,T+1,n))
    ref_states = np.loadtxt(pjoin(load_dir, "ref_states.txt")).reshape((N,T,n))
    inputs = np.loadtxt(pjoin(load_dir, "inputs.txt")).reshape((N,T,m))
    solve_times = np.loadtxt(pjoin(load_dir, "solve_times.txt")).T

    x_hist = states[:,0,:].T
    xr_hist = ref_states[:,0,:].T
    u_hist = inputs[:,0,:].T

    window = tabby.TabbedPlotWindow("Affine MPC")
    fig_track = window.add_figure_tab("states")
    fig_st = window.add_figure_tab("solve times")

    labels = ["position (m)", "velocity (m/s)", "force (N)"]

    for i in range(n):
        ax = fig_track.add_subplot(n+m,1,i+1)
        ax.plot(t_hist, x_hist[i], label="state")
        ax.plot(t_hist, xr_hist[i], "r--", label="ref")
        ax.set_ylabel(labels[i])
        if i == 0:
            ax.legend()

    for i in range(m):
        ax = fig_track.add_subplot(n+m,1,n+i+1)
        ax.plot(t_hist, u_hist[i], label="input")
        ax.set_ylabel(labels[n+i])
    ax.set_xlabel("time (s)")

    ax = fig_st.add_subplot()
    ax.plot(t_hist, solve_times[0], label="setup + opt")
    ax.plot(t_hist, solve_times[1], label="opt")
    ax.legend()
    ax.set_xlabel("sim time")
    ax.set_ylabel("solve time (s)")

    mean = np.mean(solve_times[0])
    if mean == -1.0: # user did not provide times
        mean = np.mean(solve_times[1])
    ax.set_title(f"Average solve time = {mean:.6f}s ({1/mean:.2f} Hz)")

    tabby.abracatabra(tight_layout=True)


if __name__ == "__main__":
    main()