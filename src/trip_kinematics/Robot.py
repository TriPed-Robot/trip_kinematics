
from typing import Union, List
from trip_kinematics.HomogenTransformationMatrix import TransformationMatrix
from casadi import Opti
from trip_kinematics.KinematicGroup import KinematicGroup, make_homogenous_transformation_matrix
import numpy as np


class Robot:
    """[summary]

    Returns:
        [type]: [description]
    """    """[summary]
    """

    def __init__(self, kinematic_chain: List[KinematicGroup]) -> None:
        """[summary]

        Args:
            kinematic_chain (List[KinematicChainPart]): [description]
        """
        self.__kinematic_chain = kinematic_chain

    def get_groups(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return self.__kinematic_chain


def forward_kinematics(robot: Robot):
    transformation = TransformationMatrix()
    for part in robot.get_groups():
        hmt = part.get_transformation()
        transformation = transformation * hmt
    return transformation.get_translation()


def inverse_kinematics(robot: Robot, end_effector_position):
    ''' Creating numeric equation '''

    opti = Opti()

    matrix = TransformationMatrix()

    states_to_solve_for = []

    groups = robot.get_groups()

    for group in groups:

        virtual_transformations = group.get_virtual_transformations()
        group_states = []

        for virtual_transformation in virtual_transformations:

            state = virtual_transformation.state

            for key in state.keys():

                start_value = state[key]
                state[key] = opti.variable()
                opti.set_initial(state[key], start_value)
            hmt = make_homogenous_transformation_matrix(
                virtual_transformation)
            matrix = matrix * hmt
            group_states.append(state)

        states_to_solve_for.append(group_states)

    translation = matrix.get_translation()

    equation = (translation[0] - end_effector_position[0])**2 + (translation[1] -
                                                                 end_effector_position[1])**2 + (translation[2] - end_effector_position[2])**2

    ''' Setup solver '''

    opti.minimize(equation)

    p_opts = {"print_time": False}
    s_opts = {"print_level": 0, "print_timing_statistics": "no"}

    ''' Solve '''

    opti.solver('ipopt', p_opts, s_opts)

    sol = opti.solve()

    ''' Build virtual states from solved values '''

    solved_states = []

    for state in states_to_solve_for:
        solved_state = []

        for sub_state in state:
            solved_sub_state = {}

            for key in sub_state.keys():
                solved_sub_state.setdefault(key, sol.value(sub_state[key]))

            solved_state.append(solved_sub_state)

        solved_states.append(solved_state)

    ''' Apply g mapping '''

    final_states = []       # Final states consist of actuated states and trivial states

    for i in range(len(solved_states)):

        if len(solved_states) != len(groups):
            raise RuntimeError("States non match!")

        state = solved_states[i]
        group = groups[i]

        group.set_virtual_state(state)
        actuated_state = group.get_actuated_state()

        if actuated_state != None:   # TODO: check new state logic could throw with static group
            final_states.append([actuated_state])
        else:
            final_states.append(state)

    return final_states
