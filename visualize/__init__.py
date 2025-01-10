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
import matplotlib
from matplotlib import pyplot as plt
from typing import Optional, Tuple

def setup_plot():
    matplotlib.rcParams['font.family'] = ['Liberation Sans', 'sans-serif']
    font = {'weight': 'normal','size': 15}
    matplotlib.rc('font', **font)


def plot_trajectory(
    ax,
    vessel_trajectory: Optional[Tuple],
):
    xs, ys = vessel_trajectory
    ax.set_aspect("equal")
    ax.scatter(xs[:2], ys[:2], color="red", label="Start point")
    ax.scatter(xs[-1:], ys[-1:], color="green", label="End point")
    ax.plot(xs, ys, 'orange', label="Actual trajectory")
    ax.set_xlabel("m")
    ax.set_ylabel("m")
    ax.legend(loc="lower right")