
from typing import Union, List
from trip_kinematics.HomogenTransformationMartix import HomogenousTransformationMatrix
from casadi import Opti
from trip_kinematics.KinematicGroup import KinematicGroup, make_homogenious_transformation_matrix
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


def forward_kinematic(robot: Robot):
    transformation = HomogenousTransformationMatrix()
    for part in robot.get_groups():
        hmt = part.get_transformation()
        transformation = transformation * hmt
    return transformation.get_translation()


def inverse_kinematics(robot: Robot, end_effector_position):
    opti = Opti()

    matrix = HomogenousTransformationMatrix()

    states_to_solve_for = []

    groups = robot.get_groups()

    for group in groups:
        virtual_transformations = group.get_virtual_transformations()
        group_states = []
        for v_trans in virtual_transformations:
            state = v_trans.state
            for key in state.keys():
                start_value = state[key]
                state[key] = opti.variable()
                opti.set_initial(state[key], start_value)
            hmt = make_homogenious_transformation_matrix(v_trans)
            matrix = matrix * hmt
            group_states.append(state)

        translation = matrix.get_translation()

        states_to_solve_for.append(group_states)

    equation = (translation[0] - end_effector_position[0])**2 + (translation[1] -
                                                                 end_effector_position[1])**2 + (translation[2] - end_effector_position[2])**2

    opti.minimize(equation)

    p_opts = {"print_time": False}
    s_opts = {"print_level": 0, "print_timing_statistics": "no"}

    opti.solver('ipopt', p_opts, s_opts)

    sol = opti.solve()

    before_g_mapping = []

    for state in states_to_solve_for:
        solved = []
        for sub_state in state:
            stt = {}
            for key in sub_state.keys():
                stt.setdefault(key, sol.value(sub_state[key]))
            solved.append(stt)
        before_g_mapping.append(solved)

    after_g_mapping = []

    for i in range(len(before_g_mapping)):
        if len(before_g_mapping) != len(groups):
            raise RuntimeError("States non match!")
        state = before_g_mapping[i]
        herp = groups[i]

        herp.set_state(state)
        act_state = herp.get_actuated_state()
        if act_state != None:
            after_g_mapping.append([act_state])
        else:
            after_g_mapping.append(state)
    return after_g_mapping
