import numpy as np
import mmgdynamics.calibrated_vessels as cvs

from visualize import setup_plot, plot_trajectory, plot_heading
from shipControl.los import LOSController
from mmgdynamics.maneuvers import *
from mmgdynamics.structs import Vessel
from mmgdynamics import pstep

desired_path = [
    (500, 5000),
    (5000, 7000),
]
# Use a pre-calibrated vessel
vessel = Vessel(**cvs.kvlcc2_full)

iters = 2000
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
# init rudder angle; rad
delta = 0 / 180 * np.pi
delta_limit = 45 / 180 * np.pi

controller = LOSController(
    waypoints=desired_path,
    kp=1,
    ki=1,
    kd=40,
    reached_threshold=30,
    rudder_limit=delta_limit,
)

for i in range(iters):
    u, v, r = uvr
    y, x, psi = Eta

    xs.append(x)
    ys.append(y)
    psis.append(psi/np.pi*180)

    is_ended, delta = controller.step(
        cur_pos=(x, y),
        cur_psi=psi,
    )


    if is_ended:
        print("Target reached at iter {}!".format(i))
        break

    uvr, Eta = pstep(
        X=uvr,
        pos=np.array([y, x]),
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