# encoding: utf-8
# author: vectorwang@hotmail.com
# license: MIT

r"""

"""
import numpy as np

from matplotlib import pyplot as plt
from typing import Tuple, Iterable
from visualize import plot_vo_cone

class HeadingControlVO:
    def __init__(
            self,
            os_size: float,
            obs_size: float,
    ):
        """
        own ship size;
        dynamic obs size;
        """
        self.os_size = os_size
        self.obs_size = obs_size

    def find_tangent_points(
            self,
            pt1: Tuple[float, float],
            pt2: Tuple[float, float],
            r: float,
    ):
        x1, y1 = pt1
        x2, y2 = pt2

        d = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

        theta = np.arccos(r / d)

        angle_to_center = np.arctan2(y2 - y1, x2 - x1)

        angle_t1 = angle_to_center - theta
        angle_t2 = angle_to_center + theta

        tangent_point1 = (x2 + r * np.cos(angle_t1), y2 + r * np.sin(angle_t1))
        tangent_point2 = (x2 + r * np.cos(angle_t2), y2 + r * np.sin(angle_t2))

        return (tangent_point1, tangent_point2)

    def calc_vo(
            self,
            os_loc: Tuple[float, float],
            obs_loc: Tuple[float, float],
            obs_velo: Tuple[float, float],
    ):
        """
        velo in (velo_x, velo_y)
        output: vo_corn with cone_cross_pt, line_pt1, line_pt2
        """
        virtual_size = self.os_size + self.obs_size
        original_vo_pt1, original_vo_pt2 = self.find_tangent_points(os_loc, obs_loc, virtual_size)

        pt1_x, pt1_y = original_vo_pt1
        pt2_x, pt2_y = original_vo_pt2
        velo_x, velo_y = obs_velo
        cone_cross_pt = (os_loc[0]+velo_x, os_loc[1]+velo_y)
        vo_pt1 = (pt1_x+velo_x, pt1_y+velo_y)
        vo_pt2 = (pt2_x+velo_x, pt2_y+velo_y)

        return (cone_cross_pt, vo_pt1, vo_pt2)

    def is_inside_vo_cone(
            self,
            vo_cone: Tuple[Tuple, Tuple, Tuple],
            os_loc: Tuple[float, float],
            velo_to_be_checked: Tuple[float, float],
    ) -> bool:
        cone_cross_pt, vo_pt1, vo_pt2 = vo_cone
        velo_vec = np.array([os_loc[0]+velo_to_be_checked[0], os_loc[1]+velo_to_be_checked[1]])

        velo_vec = velo_vec - np.array(cone_cross_pt)
        vo_dir1 = np.array([vo_pt1[0] - cone_cross_pt[0], vo_pt1[1] - cone_cross_pt[1]])
        vo_dir2 = np.array([vo_pt2[0] - cone_cross_pt[0], vo_pt2[1] - cone_cross_pt[1]])

        cross_1 = np.cross(vo_dir1, velo_vec)
        cross_2 = np.cross(velo_vec, vo_dir2)

        """
        if not (cross_1 >= 0  and cross_2 >= 0):
            _, ax = plt.subplots()
            plot_vo_cone(ax, vo_cone, os_loc, velo_to_be_checked)
            plt.show()
        """

        #breakpoint()
        return cross_1 >= 0 and cross_2 >= 0

    def is_inside_vo_cone_approximately(
            self,
            vo_cone: Tuple[Tuple, Tuple, Tuple],
            os_loc: Tuple[float, float],
            velo_to_be_checked: Tuple[float, float],
            margin: float=0.2,
    ) -> bool:
        """
        turning causes kvlcc2 speed to vary
        checks velocity within certain margin is inside
        """
        small_velo = np.array(velo_to_be_checked) * (1-margin)
        big_velo = np.array(velo_to_be_checked) * (1.2-margin)
        return self.is_inside_vo_cone(vo_cone, os_loc, small_velo) or self.is_inside_vo_cone(vo_cone, os_loc, velo_to_be_checked) \
            or self.is_inside_vo_cone(vo_cone, os_loc, big_velo)

        
    def calc_heading_target(
            self,
            os_loc: Tuple[float, float],
            obs_loc: Tuple[float, float],
            target_loc: Tuple[float, float],
            os_velo: Tuple[float, float],
            obs_velo: Tuple[float, float],
    ):
        """
        os for own ship
        obs for obstacle
        gives the heading that:
            avoids velocity obstacle and is nearest to the heading of target_loc
        """
        vo_cone = self.calc_vo(
            os_loc,
            obs_loc,
            obs_velo,
        )
        target_dir = np.arctan2(target_loc[1]-os_velo[1], target_loc[0]-os_velo[0])
        velo_abs = np.linalg.norm(np.array(os_velo))
        #breakpoint()

        # search for 12 times, 30 degrees each time
        for i in range(6):
            search_dir_1 = target_dir - (i*30) / 180 * np.pi
            search_dir_2 = target_dir + (i*30) / 180 * np.pi
            velo_vec_1 = (np.cos(search_dir_1) * velo_abs, np.sin(search_dir_1) * velo_abs)
            velo_vec_2 = (np.cos(search_dir_2) * velo_abs, np.sin(search_dir_2) * velo_abs)

            if not self.is_inside_vo_cone_approximately(vo_cone, os_loc, velo_vec_1):
                #print("searched", search_dir_1)
                # fig, ax = plt.subplots()
                # plot_vo_cone(ax, vo_cone, os_loc, velo_vec_1)
                # plt.show()
                return np.arctan2(velo_vec_1[0], velo_vec_1[1])
            if not self.is_inside_vo_cone_approximately(vo_cone, os_loc, velo_vec_2):
                # fig, ax = plt.subplots()
                # plot_vo_cone(ax, vo_cone, os_loc, velo_vec_2)
                # plt.show()
                return np.arctan2(velo_vec_2[0], velo_vec_2[1])
        
        # should not reach here unless already collided


class ColregVO(HeadingControlVO):
    def calc_vo(
            self,
            os_loc: Tuple[float, float],
            obs_loc: Tuple[float, float],
            obs_velo: Tuple[float, float],
    ):
        return self.calc_colreg_vo(
            os_loc,
            obs_loc,
            obs_velo,
        )

    def calc_colreg_vo(
            self,
            os_loc: Tuple[float, float],
            obs_loc: Tuple[float, float],
            obs_velo: Tuple[float, float],
    ):
        """
        velo in (velo_x, velo_y)
        output: vo_corn with cone_cross_pt, line_pt1, line_pt2
            vo area includes collision and passing from left (seeing obs on the right)
        """
        virtual_size = self.os_size + self.obs_size
        original_vo_pt1, original_vo_pt2 = self.find_tangent_points(os_loc, obs_loc, virtual_size)
        pt1_x, pt1_y = original_vo_pt1
        pt2_x, pt2_y = original_vo_pt2
        os_x, os_y = os_loc

        vec1_x = pt1_x - os_x
        vec1_y = pt1_y - os_y
        angle1 = np.arctan2(vec1_y, vec1_x)
        vec2_x = pt2_x - os_x
        vec2_y = pt2_y - os_y
        angle2 = np.arctan2(vec2_y, vec2_x)

        mid_angle = (angle1+angle2) / 2
        # crossing-with-obs-on-the-right border
        new_border_angle = mid_angle + np.pi/2
        vec2_x = np.cos(new_border_angle)
        vec2_y = np.sin(new_border_angle)
        pt2_x = vec2_x + os_x
        pt2_y = vec2_y + os_y

        velo_x, velo_y = obs_velo
        cone_cross_pt = (os_loc[0]+velo_x, os_loc[1]+velo_y)
        vo_pt1 = (pt1_x+velo_x, pt1_y+velo_y)
        vo_pt2 = (pt2_x+velo_x, pt2_y+velo_y)

        return (cone_cross_pt, vo_pt1, vo_pt2)