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

# 本船路径点
desired_path = [
    (0, 0),
    (4000, 4000),
    (8000, 4000),
]
# Use a pre-calibrated vessel
# 使用全尺寸的kvlcc2 mmg水动力系数进行模拟
vessel = Vessel(**cvs.kvlcc2_full)
# 本船和待避让船均为300m左右船长
# 此项影响碰撞判定和VO避障
os_length = 300
participant_length = 300

# 一轮1s，最多模拟5000轮
iters = 5000
xs = []
ys = []
# 艏向
psis = []

# u, v, r; m/s, m/s, rad/s
# 8 knots
uvr = np.array([8*0.514, 0, 0])
# x, y, psi; m, m, rad
Eta = np.array([0, 0, 45/180*np.pi])
# propeller revs; s⁻¹
# 设定本船保持1转每秒的螺旋桨转速
nps = 1.0
# init rudder angle; rad
# 初始舵角0度
delta = 0 / 180 * np.pi
# 设定最大舵角为45度
delta_limit = 45 / 180 * np.pi

# params for traffic participant
p_xs = []
p_ys = []
p_psis = []

# 设定待避让船从(0,4000)位置出发，艏向-45度，初始速度8节，无舵角，螺旋桨转速1直线航行
# 8 knots
p_uvr = np.array([8*0.514, 0, 0])
# y, x, psi; m, m, rad
p_Eta = np.array([0, 4000, -45/180*np.pi])
# propeller revs; s⁻¹
p_nps = 1.0
# init rudder angle; rad
p_delta = 0


# los控制器
# 简化控制，只控制舵角，保证船艏对准下一个路径点
# 因为无流无风，所以本船没有稳态的横漂，可以做此简化
# 如果有流，则应根据实际速度方向设计控制器，而非艏向
controller = LOSController(
    waypoints=desired_path,
    kp=1,
    ki=1,
    kd=40,
    reached_threshold=100,
    rudder_limit=delta_limit,
)

# 简化vo控制
# vo参考：https://zhuanlan.zhihu.com/p/662684761
# 该控制器基于VO算法，寻找一个与当前速度大小相同，但是方向改变的可行速度作为控制目标
# 输出一个预期艏向角（因为假设只尝试控制船舶速度方向
# 由于：
# 1. 虽然螺旋桨转速不变，但转向时会减速，转向完成后会加速
# 2. 全尺寸船舶惯性大，控制有迟滞
# 所以：
# 设定一个上下浮动比例，控制器选取速度时，检查该比例下大小浮动的速度是否都在可行速度域内
# 以确保选取的速度方向足够鲁棒
vo_controller = HeadingControlVO(
    os_size=os_length,
    obs_size=participant_length,
)

# vo控制器提供的预期艏向由这个pid控制器执行
vo_pid_controller = PID(
    kp=1,
    ki=1,
    kd=40,
    buffer_size=10,
)


# 当两船中心距离在9倍船长范围内时，输出True
# 此时本船终止循迹任务，启动vo开始避障
def in_collision_avoidance_range(
        os_loc,
        obs_loc,
):
    """
    have the risk of collision if distance within 9 * ship_length
    """
    os_loc = np.array(os_loc)
    obs_loc = np.array(obs_loc)
    return np.linalg.norm(os_loc-obs_loc) < 9 * os_length


# 速度大小+方向 -> 速度xy分量
def velo_2_veloxy(velo, heading):
    dir = np.pi / 2 - heading
    return (np.cos(dir) * velo, np.sin(dir) * velo)


# 每60秒启动一次vo控制器提供新的目标艏向
vo_interval = 60
in_collison_avoidance_time = 0
# 记录两船间历史最小距离
min_dist = 1e12

for i in range(iters):
    """os"""
    u, v, r = uvr
    # xy反向因为船舶坐标系与普通平面系的区别
    # mmg坐标z方向指向水深
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

    # 如果在设定范围内，终止循迹，启动vo开始避障
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
        # 计时器++
        in_collision_avoidance_time += 1
        # 获取目标舵角,如果超过舵角限制则clip
        delta = vo_pid_controller.control(vo_heading, psi)
        if delta > delta_limit:
            delta = delta_limit
        elif delta < -delta_limit:
            delta = -delta_limit

        # 碰撞，结束
        if current_dist < os_length+participant_length:
            print("Collided at iter {}!".format(i))
            print("Min dist: {}".format(min_dist))
            break

        print("VO triggered, current heading: {}, desire heading: {}.".format(psi, vo_heading))
        print("Input rudder: ", delta/np.pi*180)
    # 常规循迹流程，由los+pid组成的控制器输出控制量：舵角
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

    # 依次模拟本船和待规避船
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

# 绘图，静态
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

# 绘图，动图
fig, ax = plt.subplots()
animate_trajectory(
    fig,
    ax,
    vessel_trajectory=(xs, ys),
    global_path=desired_path,
    participant_trajectory=(p_xs, p_ys),
)