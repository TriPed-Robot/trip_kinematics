
from typing import Union, List
from trip_kinematics.HomogenTransformationMatrix import TransformationMatrix
from casadi import Opti
from trip_kinematics.KinematicGroup import KinematicGroup
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
        self.__group_dict = {}
        for group in kinematic_chain:
            self.__group_dict[str(group)]=group

    def get_groups(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return self.__group_dict

    def set_virtual_state(self, state: Dict[Dict[str, float]]):
        for key in state.keys():
            self.__group_dict[key].set_virtual_state(state[key])

    def get_symbolic_rep(self,opti_obj,endeffector):
        """This Function returnes a symbolic representation of the virtual chain.


        Returns:
            [type]: [description]
        """
        matrix = TransformationMatrix()

        symbolic_state = {}

        groups = self.get_groups()

        for group_key in groups.keys():
            group = groups[group_key]

            virtual_transformations = group.get_virtual_transformations()
            group_states = []

            for virtual_transformation in virtual_transformations:

                state = virtual_transformation.state

                for key in state.keys():

                    start_value = state[key]
                    state[key] = opti_obj.variable()
                    opti_obj.set_initial(state[key], start_value)
                hmt = virtual_transformation.get_transformation_matrix()
                matrix = matrix * hmt
                group_states.append(state)

            symbolic_state[group_key]= group_states
        return matrix, symbolic_state

    @staticmethod
    def solver_to_virtual_state(sol,symbolic_state):
        """This Function maps a opti solver solution to the virtual state of the robot

        Returns:
            [type]: [description]
        """
        solved_states = {}

        for state_key in symbolic_state.keys():
            state=symbolic_state[state_key]
            solved_state = []

            for sub_state in state:
                solved_sub_state = {}

                for key in sub_state.keys():
                    solved_sub_state.setdefault(key, sol.value(sub_state[key]))

                solved_state.append(solved_sub_state)

            solved_states[state_key] = solved_state 
        return solved_states




def forward_kinematics(robot: Robot):
    transformation = TransformationMatrix()
    for part in robot.get_groups().values():
        hmt = part.get_transformation_matrix()
        transformation = transformation * hmt
    return transformation.get_translation()


def inverse_kinematics(robot: Robot, end_effector_position):
    ''' Creating numeric equation '''

    opti = Opti()
    matrix, states_to_solve_for = robot.get_symbolic_rep(opti,"placeholder_endeffector")

    # position only inverse kinematics
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
    solved_states = robot.solver_to_virtual_state(sol,states_to_solve_for)

    robot.set_virtual_state(solved_states)
    ''' Apply g mapping '''

    final_states = []       # Final states consist of actuated states and trivial states
    groups = robot.get_groups()
    for key in solved_states.keys():


        state = solved_states[key]
        group = groups[key]

        group.set_virtual_state(state)
        actuated_state = group.get_actuated_state()

        if actuated_state != None:   # TODO: check new state logic could throw with static group
            final_states.append([actuated_state])
        else:
            final_states.append(state)

    return final_states
