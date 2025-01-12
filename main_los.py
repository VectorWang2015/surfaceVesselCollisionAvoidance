import numpy as np
import mmgdynamics.calibrated_vessels as cvs

from visualize import setup_plot, plot_trajectory, plot_heading
from shipControl.los import LOSController
from mmgdynamics.maneuvers import *
from mmgdynamics.structs import Vessel
from mmgdynamics import pstep

desired_path = [
    (0, 0),
    (1500, 7000),
    (6000, 11000),
    (26000, 15000),
]
# Use a pre-calibrated vessel
vessel = Vessel(**cvs.kvlcc2_full)

iters = 4000
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

# params for traffic participant
p_xs = []
p_ys = []
p_psis = []

# 10 knots
p_uvr = np.array([5*0.514, 0, 0])
# y, x, psi; m, m, rad
p_Eta = np.array([0, 20000, -33/180*np.pi])
# propeller revs; s⁻¹
p_nps = 1.9
# init rudder angle; rad
p_delta = 0


controller = LOSController(
    waypoints=desired_path,
    kp=1,
    ki=1,
    kd=40,
    reached_threshold=100,
    rudder_limit=delta_limit,
)

for i in range(iters):
    """os"""
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

    """participant"""
    p_u, p_v, p_r = p_uvr
    p_y, p_x, p_psi = p_Eta

    p_xs.append(p_x)
    p_ys.append(p_y)
    p_psis.append(p_psi/np.pi*180)

    p_uvr, p_Eta = pstep(
        X=p_uvr,
        pos=np.array([p_y, p_x]),
        psi=p_psi,
        vessel=vessel,
        dT=1,
        nps=p_nps,
        delta=p_delta,
    )

setup_plot()
fig, axs = plt.subplots(2)
plot_trajectory(
    ax=axs[0],
    vessel_trajectory=(xs, ys),
    global_path=desired_path,
    participant_trajectory=(p_xs, p_ys),
)
plot_heading(
    ax=axs[1],
    headings=psis,
)
plt.show()