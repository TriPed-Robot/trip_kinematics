from typing import Dict
from math import radians
import casadi
import numpy as np

from trip_kinematics.Utility import hom_rotation, get_translation
from trip_kinematics.Utility import hom_translation_matrix, y_axis_rotation_matrix
from trip_kinematics.KinematicGroup import KinematicGroup
from trip_kinematics.Transformation import Transformation
from trip_kinematics.Robot import Robot


l_1 = 1
l_2 = 0.7
l_3 = 1.2
l_4 = 0.4
l_5 = 1.7

# zero conventions for the actuated joints
a_1_offset = 0
a_2_offset = 0

virtual_joint_1 = Transformation(name="q_1",
                                 values={'ry': 0},
                                 state_variables=['ry'])
link_1 = Transformation(name="link_1",
                        values={'tx': l_1+l_3+0.4},
                        parent=virtual_joint_1)
virtual_joint_2 = Transformation(name="q_2",
                                 values={'ry': radians(-90)},
                                 state_variables=['ry'])
link_2 = Transformation(name="link_2",
                        values={'tx': l_5},
                        parent=virtual_joint_2)


################################################
# Direct mappings using geometric calculations #
################################################

def geometric_q_to_a_group_1(state: Dict[str, float], tips: Dict[str, float] = None):
    # convert joint angle to triangle angle
    q_1 = radians(90) - state['q_1']['ry']
    return {'a_1': np.sqrt(l_1**2+l_2**2-2*l_1*l_2*np.cos(q_1))}


def geometric_a_to_q_group_1(state: Dict[str, float], tips: Dict[str, float] = None):
    a_1 = state['a_1'] + a_1_offset
    return {'q_1': {'ry': np.arccos((l_1**2+l_2**2-a_1**2)/(2*l_1*l_2))}}


def geometric_q_to_a_group_2(state: Dict[str, float], tips: Dict[str, float] = None):
    q_2 = -1 * state['q_2']['ry']  # convert joint angle to triangle angle
    return {'a_2': np.sqrt(l_3**2+l_4**2-2*l_3*l_4*np.cos(q_2))}


def geometric_a_to_q_group_2(state: Dict[str, float], tips: Dict[str, float] = None):
    a_2 = state['a_2'] + a_2_offset
    return {'q_2': {'ry': np.arccos((l_3**2+l_4**2-a_2**2)/(2*l_3*l_4))}}


geometric_group_1 = KinematicGroup(name="geometric group 1",
                                   virtual_chain=[virtual_joint_1, link_1],
                                   actuated_state={'a_1': 0},
                                   actuated_to_virtual=geometric_a_to_q_group_1,
                                   virtual_to_actuated=geometric_q_to_a_group_1)

geometric_group_2 = KinematicGroup(name="geometric group 2",
                                   virtual_chain=[virtual_joint_2, link_2],
                                   actuated_state={'a_2': 0},
                                   actuated_to_virtual=geometric_a_to_q_group_2,
                                   virtual_to_actuated=geometric_q_to_a_group_2,
                                   parent=geometric_group_1)

geometric_excavator = Robot([geometric_group_1, geometric_group_2])


###################################################################
# mappings using colsure equation solution geometric calculations #
###################################################################
opts = {'ipopt.print_level': 0, 'print_time': 0}

closure_1_state = casadi.SX.sym('cls_1_q', 3)

cls_q_1 = hom_rotation(y_axis_rotation_matrix(closure_1_state[0]))
cls_l_1 = hom_translation_matrix(t_x=l_1)
cls_qs_2 = hom_rotation(y_axis_rotation_matrix(closure_1_state[1]))
cls_a_1 = hom_translation_matrix(t_x=closure_1_state[2])
cls_a_1z = hom_translation_matrix(t_x=a_1_offset)
cls_1_trafo = cls_q_1 * cls_l_1 * cls_qs_2 * cls_a_1 * cls_a_1z

cls_1_trafo_pos = get_translation(cls_1_trafo)

c_1 = (cls_1_trafo_pos[0]-l_2)**2 + \
    cls_1_trafo_pos[1]**2 + cls_1_trafo_pos[2]**2


def closure_q_to_a_group_1(state: Dict[str, float]):
    nlp = {'x': closure_1_state[1:], 'f': c_1, 'p': closure_1_state[0]}
    nlp_solver = casadi.nlpsol('q_to_a', 'ipopt', nlp, opts)
    solution = nlp_solver(x0=[0, 0], p=[state['q_1']['ry']])
    sol_vector = np.array(solution['x'])
    return {'a_1': sol_vector[1]}


def closure_a_to_q_group_1(state: Dict[str, float]):
    nlp = {'x': closure_1_state[:1], 'f': c_1, 'p': closure_1_state[2]}
    nlp_solver = casadi.nlpsol('a_to_q', 'ipopt', nlp, opts)
    solution = nlp_solver(x0=[0, 0], p=[state['a_1']])
    sol_vector = np.array(solution['x'])
    return {'q_1': {'ry': sol_vector[0]}}


closure_2_state = casadi.SX.sym('cls_2_q', 3)

cls_q_2 = hom_rotation(y_axis_rotation_matrix(closure_2_state[0]))
cls_l_4 = hom_translation_matrix(t_x=l_4)
cls_qs_4 = hom_rotation(y_axis_rotation_matrix(closure_2_state[1]))
cls_a_2 = hom_translation_matrix(t_x=closure_2_state[2])
cls_a_2z = hom_translation_matrix(t_x=a_1_offset)
cls_2_trafo = cls_q_2 * cls_l_4 * cls_qs_4 * cls_a_2 * cls_a_2z

cls_2_trafo_pos = get_translation(cls_1_trafo)

c_2 = (cls_2_trafo_pos[0]+l_3)**2 + \
    cls_2_trafo_pos[1]**2 + cls_2_trafo_pos[2]**2


def closure_q_to_a_group_1(state: Dict[str, float]):
    nlp = {'x': closure_2_state[1:], 'f': c_2, 'p': closure_2_state[0]}
    nlp_solver = casadi.nlpsol('q_to_a', 'ipopt', nlp, opts)
    solution = nlp_solver(x0=[0, 0], p=[state['q_1']['ry']])
    sol_vector = np.array(solution['x'])
    return {'a_2': sol_vector[1]}


def closure_a_to_q_group_1(state: Dict[str, float]):
    nlp = {'x': closure_2_state[:1], 'f': c_2, 'p': closure_2_state[2]}
    nlp_solver = casadi.nlpsol('a_to_q', 'ipopt', nlp, opts)
    solution = nlp_solver(x0=[0, 0], p=[state['a_1']])
    sol_vector = np.array(solution['x'])
    return {'q_2': {'ry': sol_vector[0]}}


closure_group_1 = KinematicGroup(name="geometric group 1",
                                 virtual_chain=[virtual_joint_1, link_1],
                                 actuated_state={'a_1': 0},
                                 actuated_to_virtual=geometric_a_to_q_group_1,
                                 virtual_to_actuated=geometric_q_to_a_group_1)

closure_group_2 = KinematicGroup(name="geometric group 2",
                                 virtual_chain=[virtual_joint_2, link_2],
                                 actuated_state={'a_2': 0},
                                 actuated_to_virtual=geometric_a_to_q_group_2,
                                 virtual_to_actuated=geometric_q_to_a_group_2,
                                 parent=geometric_group_1)

closure_excavator = Robot([closure_group_1, closure_group_2])
