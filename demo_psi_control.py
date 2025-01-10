import numpy as np
import mmgdynamics.calibrated_vessels as cvs

from visualize import setup_plot, plot_trajectory, plot_heading
from shipControl.pid import PID
from mmgdynamics.maneuvers import *
from mmgdynamics.structs import Vessel
from mmgdynamics import pstep

# Use a pre-calibrated vessel
vessel = Vessel(**cvs.kvlcc2_full)

iters = 600
xs = []
ys = []
psis = []

# desire psi 120 degrees
desire_psi = 120 / 180 * np.pi

# u, v, r; m/s, m/s, rad/s
# 10 knots
uvr = np.array([10*0.514, 0, 0])
# x, y, psi; m, m, rad
Eta = np.array([0, 0, 0])
# propeller revs; s⁻¹
nps = 3.0
# rudder angle; rad
delta = 0 / 180 * np.pi
delta_limit = 45 / 180 * np.pi

controller = PID(
    kp=1,
    ki=1,
    kd=40,
)

for _ in range(iters):
    u, v, r = uvr
    x, y, psi = Eta

    xs.append(y)
    ys.append(x)
    psis.append(psi/np.pi*180)

    desire_delta = controller.control(desire_value=desire_psi, current_value=psi)

    # clamp input rudder
    delta = delta_limit if desire_delta > delta_limit else desire_delta
    delta = -delta_limit if desire_delta < -delta_limit else delta

    uvr, Eta = pstep(
        X=uvr,
        pos=np.array([x, y]),
        psi=psi,
        vessel=vessel,
        dT=1,
        nps=nps,
        delta=delta,
    )

setup_plot()
fig, axs = plt.subplots(2)
plot_trajectory(
    ax=axs[0],
    vessel_trajectory=(xs, ys),
)
plot_heading(
    ax=axs[1],
    headings=psis,
)

plt.show()