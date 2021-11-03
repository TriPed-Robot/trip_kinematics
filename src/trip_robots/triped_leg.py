from math import radians
from typing import Dict
from casadi import SX, nlpsol, vertcat
import numpy as np


from trip_kinematics.KinematicGroup import KinematicGroup
from trip_kinematics.Transformation import Transformation
from trip_kinematics.Robot import Robot
from trip_kinematics.Utility import hom_translation_matrix, x_axis_rotation_matrix
from trip_kinematics.Utility import y_axis_rotation_matrix, z_axis_rotation_matrix
from trip_kinematics.Utility import hom_rotation, get_translation


def sphere_centers(r_x, r_y, r_z):
    """Generates the centers of the spheres used for the geometric closure equation

    Args:
        r_x (float): gimbal joint state rx
        r_y (float): gimbal joint state ry
        r_z (float): gimbal joint state rz

    Returns:
        numpy array, numpy array: Two 3D arrays containing the sphere centers
    """
    a_ccs_p_trans_m = hom_translation_matrix(
        t_x=0.265, t_y=0, t_z=0.014)
    a_ccs_p_rot_m = hom_rotation(x_axis_rotation_matrix(r_x) @
                                 y_axis_rotation_matrix(r_y) @
                                 z_axis_rotation_matrix(r_z))
    a_p_sph_1_2 = hom_translation_matrix(
        t_x=0.015, t_y=0.029, t_z=-0.0965)
    a_p_sph_2_2 = hom_translation_matrix(
        t_x=0.015, t_y=-0.029, t_z=-0.0965)

    a_ccs_ = a_ccs_p_trans_m @ a_ccs_p_rot_m
    a_c1 = a_ccs_ @ a_p_sph_1_2
    a_c2 = a_ccs_ @ a_p_sph_2_2

    return get_translation(a_c1), get_translation(a_c2)


def intersection_left(theta):
    """calculates the desired sphere intersection point based on the left swing joint

    Args:
        theta (float): angle of the left swing joint

    Returns:
        numpy array: the sphere intersection
    """
    a_ccs_lsm_trans = hom_translation_matrix(
        t_x=0.139807669447128, t_y=0.0549998406976098, t_z=-0.051)
    a_ccs_lsm_rot = hom_rotation(z_axis_rotation_matrix(radians(-345.0)))
    a_mcs_1_joint = hom_rotation(z_axis_rotation_matrix(theta))
    a_mcs_1_sp_1_1 = hom_translation_matrix(
        t_x=0.085, t_y=0, t_z=-0.0245)

    a_ccs_sp_1_1 = a_ccs_lsm_trans @ a_ccs_lsm_rot @ a_mcs_1_joint @ a_mcs_1_sp_1_1
    return get_translation(a_ccs_sp_1_1)


def intersection_right(theta):
    """calculates the desired sphere intersection point based on the right swing joint

    Args:
        theta (float): angle of the right swing joint

    Returns:
        numpy array: the sphere intersection
    """
    a_ccs_rsm_tran = hom_translation_matrix(
        t_x=0.139807669447128, t_y=-0.0549998406976098, t_z=-0.051)
    a_ccs_rsm_rot = hom_rotation(z_axis_rotation_matrix(radians(-15.0)))
    a_mcs_2_joint = hom_rotation(z_axis_rotation_matrix(theta))
    a_mcs_2_sp_2_1 = hom_translation_matrix(
        t_x=0.085, t_y=0, t_z=-0.0245)

    a_ccs_sp_2_1 = a_ccs_rsm_tran @ a_ccs_rsm_rot @ a_mcs_2_joint @ a_mcs_2_sp_2_1
    return get_translation(a_ccs_sp_2_1)


def swing_to_gimbal(state: Dict[str, float], tips: Dict[str, float] = None):
    """Actuated to virtual state mapping for the TriPed legss closed subchain

    Args:
        state (Dict[str, float]): actuated state of the TriPed leg closed subchain
        tips (Dict[str, float], optional): Initial state for the closure equation solver.
                                           Defaults to None in which case [0,0,0] is used.

    Returns:
        Dict[str, Dict[str, float]]: the correspdonding state of the virtual chain
    """
    x_0 = [0, 0, 0]
    if tips:
        x_0[2] = tips['rx']
        x_0[3] = tips['ry']
        x_0[4] = tips['rz']

    nlp = {'x': virtual_state, 'f': closing_equation, 'p': actuated_state}
    nlp_solver = nlpsol('swing_to_gimbal', 'ipopt', nlp, opts)
    solution = nlp_solver(x0=x_0,
                          p=[state['swing_left'], state['swing_right']])
    sol_vector = np.array(solution['x'])
    return {'gimbal_joint': {'rx': sol_vector[0][0],
                             'ry': sol_vector[1][0],
                             'rz': sol_vector[2][0]}}


def gimbal_to_swing(state: Dict[str, Dict[str, float]], tips: Dict[str, float] = None):
    """Virtual to actuated state mapping for the TriPed legss closed subchain

    Args:
        state (Dict[str, Dict[str, float]]): virtual state of the TriPed leg closed subchain
        tips (Dict[str, float], optional): Initial state for the closure equation solver.
                                           Defaults to None in which case [0,0] is used.

    Returns:
        Dict[str, float]: the correspdonding actuated state
    """
    x_0 = [0, 0]
    continuity = 0
    if tips:
        x_0[0] = tips['swing_left']
        x_0[1] = tips['swing_right']
        continuity = (x_0-actuated_state).T @ (x_0-actuated_state)

    nlp = {'x': actuated_state, 'f': closing_equation+continuity, 'p': virtual_state,
           'g': actuated_state[0]*actuated_state[1]}
    nlp_solver = nlpsol('gimbal_to_swing', 'ipopt', nlp, opts)
    solution = nlp_solver(x0=x_0,
                          p=[state['gimbal_joint']['rx'],
                             state['gimbal_joint']['ry'],
                             state['gimbal_joint']['rz']],
                          ubg=0)
    sol_vector = np.array(solution['x'])
    return {'swing_left': sol_vector[0][0], 'swing_right': sol_vector[1][0]}


theta_left = SX.sym('theta_left')
theta_right = SX.sym('theta_right')
gimbal_x = SX.sym('gimbal_x')
gimbal_y = SX.sym('gimbal_y')
gimbal_z = SX.sym('gimbal_z')

virtual_state = vertcat(gimbal_x, gimbal_y, gimbal_z)
actuated_state = vertcat(theta_left, theta_right)

opts = {'ipopt.print_level': 0, 'print_time': 0}
RADIUS = 0.11
c1, c2 = sphere_centers(r_x=gimbal_x, r_y=gimbal_y, r_z=gimbal_z)
closing_equation = (((c1-intersection_left(theta_right)).T @ (c1-intersection_left(theta_right)) -
                    RADIUS**2)**2 +
                    ((c2-intersection_right(theta_left)).T @ (c2-intersection_right(theta_left)) -
                    RADIUS**2)**2)

a_ccs_p_trans = Transformation(name='A_ccs_P_trans',
                               values={'tx': 0.265, 'tz': 0.014})
a_ccs_p_rot = Transformation(name='gimbal_joint',
                             values={'rx': 0, 'ry': 0, 'rz': 0},
                             state_variables=['rx', 'ry', 'rz'],
                             parent=a_ccs_p_trans)

closed_chain = KinematicGroup(name='closed_chain',
                              virtual_chain=[a_ccs_p_trans, a_ccs_p_rot],
                              actuated_state={
                                  'swing_left': 0, 'swing_right': 0},
                              actuated_to_virtual=swing_to_gimbal,
                              virtual_to_actuated=gimbal_to_swing)

a_p_ll = Transformation(name='A_P_LL',
                        values={'tx': 1.640, 'tz': -0.037, },
                        parent=closed_chain)
a_ll_zero = Transformation(name='zero_angle_convention',
                           values={'ry': radians(-3)},
                           parent=a_p_ll)
a_ll_zero_ll_joint = Transformation(name='extend_joint',
                                    values={'ry': 0},
                                    state_variables=['ry'],
                                    parent=a_ll_zero)
a_ll_joint_fcs = Transformation(name='A_LL_Joint_FCS',
                                values={'tx': -1.5},
                                parent=a_ll_zero_ll_joint)


triped_leg = Robot([closed_chain, a_p_ll, a_ll_zero,
                   a_ll_zero_ll_joint, a_ll_joint_fcs])
triped_leg.set_actuated_state({'swing_left': 0, 'swing_right': 0})
