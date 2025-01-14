# encoding: utf-8
# author: vectorwang@hotmail.com
# license: MIT

r"""
       _                 _ _         
__   _(_)___ _   _  __ _| (_)_______ 
\ \ / / / __| | | |/ _` | | |_  / _ \
 \ V /| \__ \ |_| | (_| | | |/ /  __/
  \_/ |_|___/\__,_|\__,_|_|_/___\___|

"""
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from typing import Optional, Tuple, Iterable

def setup_plot():
    matplotlib.rcParams['font.family'] = ['Liberation Sans', 'sans-serif']
    font = {'weight': 'normal','size': 15}
    matplotlib.rc('font', **font)


def plot_trajectory(
        ax,
        vessel_trajectory: Tuple[Iterable],
        global_path: Optional[Iterable[Tuple[float, float]]]=None,
        participant_trajectory: Optional[Tuple[Iterable]]=None,
):
    xs, ys = vessel_trajectory
    ax.set_aspect("equal")
    ax.scatter(xs[:1], ys[:1], color="red", label="Start point")
    ax.scatter(xs[-1:], ys[-1:], color="green", label="End point")
    ax.plot(xs, ys, 'orange', label="Actual trajectory")
    if global_path is not None:
        xs = [x for x, _ in global_path]
        ys = [y for _, y in global_path]
        ax.plot(xs, ys, 'x--', color='grey', label="Reference path")
    if participant_trajectory is not None:
        xs, ys = participant_trajectory
        ax.plot(xs, ys, '--', color='blue', label="Traffic participant trajectory")
        ax.scatter(xs[:1], ys[:1], color="red")
        ax.scatter(xs[-1:], ys[-1:], color="green")
    ax.set_xlabel("m")
    ax.set_ylabel("m")
    ax.legend(loc="lower right")


def plot_heading(
        ax,
        headings: Iterable[float],
):
    """
    assume 1 sample/s
    """
    ax.plot(np.array(range(len(headings))), headings, label="OS")
    ax.grid()
    ax.set_xlabel("Sample time [s]")
    ax.set_ylabel("Heading [degree]")
    ax.legend(loc="lower right")


def plot_vo_cone(
        ax,
        vo_cone: Tuple[Tuple, Tuple, Tuple],
        os_loc: Tuple[float, float],
        os_velo: Tuple[float, float],
):
    cone_cross_pt, vo_pt1, vo_pt2 = vo_cone
    cross_x, cross_y = cone_cross_pt
    vo_x1, vo_y1 = vo_pt1
    vo_x2, vo_y2 = vo_pt2
    os_x, os_y = os_loc
    velo_x, velo_y = os_velo

    ax.set_aspect("equal")
    ax.set_xlim(os_loc[0]-50, os_loc[0]+50)
    ax.set_ylim(os_loc[1]-50, os_loc[1]+50)
    ax.plot([cross_x, vo_x1], [cross_y, vo_y1], "red")
    ax.plot([cross_x, vo_x2], [cross_y, vo_y2], "blue")
    ax.plot([os_x, os_x+velo_x], [os_y, os_y+velo_y], "black")
    ax.scatter([os_x], [os_y], color="black")

