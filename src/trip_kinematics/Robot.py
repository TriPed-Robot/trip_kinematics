
from typing import Union, List
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix
from casadi import Opti
from trip_kinematics.KinematicGroup import KinematicGroup


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

    def get_parts(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return self.__kinematic_chain


def forward_kinematic(robot: Robot):
    transformation = Homogenous_transformation_matrix()
    for part in robot.get_parts():
        hmt = part.get_transformation()
        transformation = transformation * hmt
    return transformation.get_translation()


def inverse_kinematics(robot: Robot, end_effector_position):
    opti = Opti()

    matrix = Homogenous_transformation_matrix()

    states_to_solve_for = []

    parts = robot.get_parts()

    for part in parts:

        state = part.get_virtual_state()
        for sub_state in state:
            for key in sub_state.keys():
                sub_state[key] = opti.variable()

        part.set_state(state)
        matrix = matrix * part.get_transformation()
        states_to_solve_for.append(state)

    translation = matrix.get_translation()

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
        if len(before_g_mapping) != len(parts):
            raise RuntimeError("States non match!")
        state = before_g_mapping[i]
        herp = parts[i]

        herp.set_state(state)
        act_state = herp.get_actuated_state()
        if act_state != None:
            after_g_mapping.append([act_state])
        else:
            after_g_mapping.append(state)
    return after_g_mapping
