from typing import Dict, List, Callable, Union
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
        self.__group_dict      = {}
        self.__actuator_group_mapping = {}
        self.__virtual_group_mapping = {}

        for group in kinematic_chain:
            self.__group_dict[str(group)]=group

            group_actuators = group.get_actuated_state()[0].keys()
            for key in group_actuators:
                if key in self.__actuator_group_mapping.keys():
                    raise KeyError("More than one robot actuated state has the same name! Please give each actuated state a unique name")
                self.__actuator_group_mapping[key]=str(group)

            group_virtuals = []
            for i in range(len(group.get_virtual_state())):
                group_virtuals.extend(list(group.get_virtual_state()[i].keys()) )
            print(group_virtuals)
            for key in group_virtuals:
                if key in self.__virtual_group_mapping.keys():
                    raise KeyError("More than one robot virtual state has the same name! Please give each virtual state a unique name")
                self.__virtual_group_mapping[key]=str(group)


    def get_groups(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return self.__group_dict

    def set_virtual_state(self, state: Dict[str, float]):
        for key in state.keys():
            corresponding_group = self.__virtual_group_mapping[key]
            self.__group_dict[corresponding_group].set_virtual_state(state[key])

    def set_actuated_state(self, state: Dict[str, float]):
        for key in state.keys():
            corresponding_group = self.__actuator_group_mapping[key]
            self.__group_dict[corresponding_group].set_actuated_state(state[key])
    
    def get_actuated_state(self):
        actuated_state=[]
        for key in self.__group_dict.keys():
            actuated_state.append([self.__group_dict[key].get_actuated_state()])
        return actuated_state

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


            for virtual_transformation in virtual_transformations:

                state = virtual_transformation.state

                for key in state.keys():

                    start_value = state[key]
                    state[key] = opti_obj.variable()
                    opti_obj.set_initial(state[key], start_value)
                    symbolic_state[key]=state[key]

                hmt = virtual_transformation.get_transformation_matrix()
                matrix = matrix * hmt

        return matrix, symbolic_state

    @staticmethod
    def solver_to_virtual_state(sol,symbolic_state):
        """This Function maps a opti solver solution to the virtual state of the robot

        Returns:
            [type]: [description]
        """
        solved_states = {}

        for key in symbolic_state.keys():
            solved_states[key]=sol.value(symbolic_state[key])
            '''
            state=symbolic_state[state_key]
            solved_state = []

            for sub_state in state:
                solved_sub_state = {}

                for key in sub_state.keys():
                    solved_sub_state.setdefault(key, sol.value(sub_state[key]))

                solved_state.append(solved_sub_state)

            solved_states[state_key] = solved_state 
            '''
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
    equation = ((translation[0] - end_effector_position[0])**2 + 
                (translation[1] - end_effector_position[1])**2 + 
                (translation[2] - end_effector_position[2])**2)

    ''' Setup solver '''
    opti.minimize(equation)
    p_opts = {"print_time": False}
    s_opts = {"print_level": 0, "print_timing_statistics": "no"}

    ''' Solve '''
    opti.solver('ipopt', p_opts, s_opts)
    sol = opti.solve()

    ''' Build actuated states from solved values '''
    solved_states = robot.solver_to_virtual_state(sol,states_to_solve_for)
    robot.set_virtual_state(solved_states)
    actuated_state = robot.get_actuated_state()
 
    return actuated_state
