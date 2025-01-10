# encoding: utf-8
# author: vectorwang@hotmail.com
# license: MIT

r"""
 _ _ 
| / |
| | |
| | |
|_|_|

"""
import numpy as np
from typing import Tuple


class L1Controller:
    """
    to be continued
    """
    def calc_l1_target(
            self,
            guide_line_dist: float,
            current_pos: Tuple[float, float],
            segment: Tuple[Tuple[float, float], Tuple[float, float]],
    ):
        """
        return a target point using L1 trajectory following algor
        segment should be a line from the desired trajectory, a tuple of start and end point
        should be guided to start or end point if each pt on the desired path is beyond the guide distance
        """
        line_pt1 = np.array(segment[0])
        line_pt2 = np.array(segment[1])
        l2 = np.sum((line_pt2-line_pt1)**2)
    
        line_vec = (line_pt2 - line_pt1) / np.sqrt(l2)

        proportion = np.dot(line_pt2-line_pt1, current_pos-line_pt1) / l2
        perpendicular_pt = proportion * (line_pt2 - line_pt1) + line_pt1
        dist = np.linalg.norm(perpendicular_pt-current_pos)

        #breakpoint()
        if dist >= guide_line_dist:
            if proportion <= 0:
                return line_pt1
            elif proportion >= 1:
                return line_pt2
            else:
                return perpendicular_pt

        elif dist < guide_line_dist:
            remaining_length = np.sqrt(guide_line_dist**2 - dist**2)
            remaining_vec = remaining_length * line_vec
            target_pt = perpendicular_pt + remaining_vec
            target_pt_proportion = np.dot(target_pt - line_pt1, line_vec) / np.sqrt(l2)
            if target_pt_proportion <= 0:
                return line_pt1
            elif target_pt_proportion >= 1:
                return line_pt2
            else:
                return target_pt

        else:
            raise Exception