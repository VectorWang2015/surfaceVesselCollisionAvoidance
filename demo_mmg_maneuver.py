import numpy as np
import mmgdynamics.calibrated_vessels as cvs

from visualize import setup_plot, plot_trajectory
from mmgdynamics.maneuvers import *
from mmgdynamics.structs import Vessel
from mmgdynamics import pstep

# Use a pre-calibrated vessel
vessel = Vessel(**cvs.kvlcc2_full)

iters = 3000
xs = []
ys = []

# u, v, r; m/s, m/s, rad/s
uvr = np.array([3.85, 0, 0])
# x, y, psi; m, m, rad
Eta = np.array([0, 0, 0])
# propeller revs; s⁻¹
nps = 1.05
# rudder angle; rad
delta = 30 / 180 * np.pi

for _ in range(iters):
    u, v, r = uvr
    x, y, psi = Eta

    xs.append(y)
    ys.append(x)

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
fig, ax = plt.subplots()
plot_trajectory(
    ax=ax,
    vessel_trajectory=(xs, ys),
)
plt.show()