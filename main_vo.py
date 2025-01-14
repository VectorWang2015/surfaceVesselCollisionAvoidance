import numpy as np
import mmgdynamics.calibrated_vessels as cvs

from visualize import setup_plot, plot_trajectory, plot_heading
from visualize.animation import animate_trajectory
from shipControl.los import LOSController
from shipControl.pid import PID
from shipControl.vo import HeadingControlVO
from mmgdynamics.maneuvers import *
from mmgdynamics.structs import Vessel
from mmgdynamics import pstep

desired_path = [
    (0, 0),
    (5000, 5000),
]
# Use a pre-calibrated vessel
vessel = Vessel(**cvs.kvlcc2_full)
os_length = 300
participant_length = 300

iters = 3000
xs = []
ys = []
psis = []

# desire psi 120 degrees
desire_psi = 120 / 180 * np.pi

# u, v, r; m/s, m/s, rad/s
# 3 knots
uvr = np.array([3*0.514, 0, 0])
# x, y, psi; m, m, rad
Eta = np.array([0, 0, 45/180*np.pi])
# propeller revs; s⁻¹
nps = 1.0
# init rudder angle; rad
delta = 0 / 180 * np.pi
delta_limit = 45 / 180 * np.pi

# params for traffic participant
p_xs = []
p_ys = []
p_psis = []

# 3 knots
p_uvr = np.array([3*0.514, 0, 0])
# y, x, psi; m, m, rad
p_Eta = np.array([0, 5000, -45/180*np.pi])
# propeller revs; s⁻¹
p_nps = 1.0
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

vo_controller = HeadingControlVO(
    os_size=os_length,
    obs_size=participant_length,
)

vo_pid_controller = PID(
    kp=1,
    ki=1,
    kd=40,
    buffer_size=10,
)


def in_collision_avoidance_range(
        os_loc,
        obs_loc,
):
    """
    have the risk of collision if distance within 6 * ship_length
    """
    os_loc = np.array(os_loc)
    obs_loc = np.array(obs_loc)
    return np.linalg.norm(os_loc-obs_loc) < 6 * os_length


def velo_2_veloxy(velo, heading):
    dir = np.pi / 2 - heading
    return (np.cos(dir) * velo, np.sin(dir) * velo)

vo_interval = 10
in_collison_avoidance_time = 0
min_dist = 1e6

for i in range(iters):
    """os"""
    u, v, r = uvr
    y, x, psi = Eta

    xs.append(x)
    ys.append(y)
    psis.append(psi/np.pi*180)

    """participant"""
    p_u, p_v, p_r = p_uvr
    p_y, p_x, p_psi = p_Eta

    p_xs.append(p_x)
    p_ys.append(p_y)
    p_psis.append(p_psi/np.pi*180)

    current_dist = np.linalg.norm((x-p_x, y-p_y))
    if current_dist < min_dist:
        min_dist = current_dist

    if in_collision_avoidance_range((x, y), (p_x, p_y)):
        # trigger vo, calc new vo target each vo_interval seconds
        if in_collison_avoidance_time % vo_interval == 0:
            os_velo = velo_2_veloxy(u, psi)
            obs_velo = velo_2_veloxy(p_u, p_psi)
            vo_heading = vo_controller.calc_heading_target(
                os_loc=(x, y),
                obs_loc=(p_x, p_y),
                target_loc=controller.current_target,
                os_velo=os_velo,
                obs_velo=obs_velo,
            )
        in_collision_avoidance_time += 1
        delta = vo_pid_controller.control(vo_heading, psi)

        if current_dist < os_length+participant_length:
            print("Collided at iter {}!".format(i))
            print("Min dist: {}".format(min_dist))
            break

        print("VO triggered, new direction: {}".format(vo_heading))
    else:
        in_collision_avoidance_time = 0
        # los control
        is_ended, delta = controller.step(
            cur_pos=(x, y),
            cur_psi=psi,
        )

        if is_ended:
            print("Target reached at iter {}!".format(i))
            print("Min dist: {}".format(min_dist))
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
"""
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
"""

fig, ax = plt.subplots()
animate_trajectory(
    fig,
    ax,
    vessel_trajectory=(xs, ys),
    global_path=desired_path,
    participant_trajectory=(p_xs, p_ys),
)