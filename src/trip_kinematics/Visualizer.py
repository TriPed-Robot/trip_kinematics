"""
Visualizer.py
This is a program to visualize a Robot class instance.
"""
from typing import Tuple
import matplotlib.pyplot as plt


from trip_robots.excavator_rr import geometric_excavator
from trip_robots.triped_leg import triped_leg
from trip_robots.triped import triped
from trip_kinematics.Robot import Robot
from trip_kinematics.Transformation import Transformation
from trip_kinematics.Utility import get_translation, identity_transformation
from trip_kinematics.KinematicGroup import KinematicGroup


DEFAULT_COLOR_PALETTE = ('#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                         '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf')


class Visualizer:

    def __init__(self, robot: Robot, color_palette: Tuple[str] = DEFAULT_COLOR_PALETTE):
        self.__color_palette = color_palette
        self.__offset_and_level_per_group = self.__get_offset_and_lvl_for_every_group(
            robot)

    def __pick_color(self, level):
        return self.__color_palette[level % len(self.__color_palette)]

    def get_coordinates_for_virtual_chain_of_group(self, group: KinematicGroup, offset: Transformation = None):
        x_coordinates = []
        y_coordinates = []
        z_coordinates = []
        transformation = identity_transformation()
        if len(offset) > 0:
            transformation = offset
            x, y, z = get_translation(offset)
            x_coordinates.append(x)
            y_coordinates.append(y)
            z_coordinates.append(z)

        virtual_chain = group.get_virtual_chain()
        for part in virtual_chain.values():
            transformation = transformation @ part.get_transformation_matrix()
            x_coordinate, y_coordinate, z_coordinate = get_translation(
                transformation)
            x_coordinates.append(x_coordinate)
            y_coordinates.append(y_coordinate)
            z_coordinates.append(z_coordinate)
        return x_coordinates, y_coordinates, z_coordinates

    def __get_offset_and_lvl_for_every_group(self, robot: Robot):
        groups = robot.get_groups()
        # find base
        bases = []
        for group in groups.values():
            if group.get_name() == group.parent:
                bases.append(group.get_name())

        que = [*bases]
        visited = set()
        current = bases[0]
        offset_and_level_per_group = dict()
        for base in bases:
            offset_and_level_per_group[base] = (0, identity_transformation())

        while que:
            # skip leaves & already visited knots
            if len(groups[current].children) > 0 and current not in visited:
                children = groups[current].children
                next_transformation = groups[current].get_transformation_matrix(
                )

                que.extend(children)

                current_level, current_transformation = offset_and_level_per_group[current]

                for child in children:
                    offset_and_level_per_group[child] = (
                        current_level + 1, current_transformation @ next_transformation)
            visited.add(current)
            if len(que) > 1:
                current = que[1]
            que.pop(0)

        return offset_and_level_per_group

    def visualize_robot(self, robot: Robot):
        ax = plt.axes(projection='3d')

        groups = robot.get_groups()

        for group in groups.values():
            current_group_level, current_group_offset = self.__offset_and_level_per_group[group.get_name(
            )]

            x, y, z = self.get_coordinates_for_virtual_chain_of_group(
                group, current_group_offset)

            color = self.__pick_color(current_group_level)
            ax.plot3D(
                x, y, z, color,  marker='.')
        plt.show()
