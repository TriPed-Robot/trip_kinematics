
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

    sorted_chain = sort_kinematic_chain_parts(robot.get_parts())

    sub_states_solved_for = []

    matrix = Homogenous_transformation_matrix()   # Identity matrix

    for part in sorted_chain:
        if isinstance(part, KinematicObject):
            if len(part.get_state().keys()) == 0:    # No state present --> link

                matrix = matrix * part.get_transformation()

            else:   # state present --> joint

                state = part.get_state()

                for key in part.get_state().keys():
                    state[key] = opti.variable()

                part.set_state(state)

                matrix = matrix * part.get_transformation()

                sub_states_solved_for.append(state)

        elif isinstance(part, KinematicGroup):

            state = part.get_state()

            for sub_state in state:
                for key in sub_state.keys():
                    sub_state[key] = opti.variable()

            part.set_state(state)

            matrix = matrix * part.get_transformation()

            sub_states_solved_for.append(state)

    translation = matrix.get_translation()

    equation = (translation[0] - end_effector_position[0])**2 + (translation[1] -
                                                                 end_effector_position[1])**2 + (translation[2] - end_effector_position[2])**2

    opti.minimize(equation)

    p_opts = {"print_time": False}
    s_opts = {"print_level": 5, "print_timing_statistics": "no"}

    opti.solver('ipopt', p_opts, s_opts)

    sol = opti.solve()

    out = []

    for state in sub_states_solved_for:
        if isinstance(state, list):
            solved = []
            for sub_state in state:
                stt = {}
                for key in sub_state.keys():
                    stt.setdefault(key, sol.value(sub_state[key]))
                solved.append(stt)
            out.append(solved)

        elif isinstance(state, dict):
            stt = {}
            for key in state.keys():
                stt.setdefault(key, sol.value(state[key]))
            out.append(stt)
        else:
            raise RuntimeError("Well shit!")

    return out
