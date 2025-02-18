import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Tuple, Iterable, Optional
from functools import partial

def update(
        frame,
        vessel_trajectory: Tuple[Iterable],
        participant_trajectory: Tuple[Iterable],
        vessel_line,
        participant_line,
        global_line,
):
    vessel_x, vessel_y = [], []
    for i in range(frame+1):
        vessel_x.append(vessel_trajectory[0][i])
        vessel_y.append(vessel_trajectory[1][i])

    participant_x, participant_y = [], []
    for i in range(frame+1):
        participant_x.append(participant_trajectory[0][i])
        participant_y.append(participant_trajectory[1][i])

    vessel_line.set_data(vessel_x, vessel_y)
    participant_line.set_data(participant_x, participant_y)

    return global_line, participant_line, vessel_line



def animate_trajectory(
        fig,
        ax,
        vessel_trajectory: Tuple[Iterable],
        global_path: Iterable[Tuple[float, float]],
        participant_trajectory: Tuple[Iterable],
        save_loc: Optional[str]=None,
):
    ax.set_aspect("equal")

    xs = [x for x, _ in global_path]
    ys = [y for _, y in global_path]

    line_reference, = ax.plot(xs, ys, 'x--', color='grey', label="Reference path")
    line_participant, = ax.plot([], [], '--', color='blue', label="Traffic participant trajectory")
    line_trajectory, = ax.plot([], [], 'orange', label="Actual trajectory")

    animate_init_fn = lambda :(line_trajectory, line_participant, line_reference)
    animate_update_fn = partial(
        update,
        vessel_trajectory=vessel_trajectory,
        participant_trajectory=participant_trajectory,
        vessel_line=line_trajectory,
        participant_line=line_participant,
        global_line=line_reference,
    )

    ani = FuncAnimation(
        fig, animate_update_fn,
        frames=len(participant_trajectory[1]),
        init_func=animate_init_fn, blit=True,
        interval=10,
    )

    if save_loc is not None:
        ani.save(save_loc, writer="pillow")

    plt.show()