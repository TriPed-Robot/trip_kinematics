from trip_kinematics.KinematicGroup import KinematicGroup, Transformation
from trip_kinematics.Robot import Robot
from casadi import  SX, nlpsol, vertcat
from typing import Dict
from trip_kinematics.Utility import hom_translation_matrix, x_axis_rotation_matrix, y_axis_rotation_matrix, z_axis_rotation_matrix, hom_rotation, get_translation
import numpy as np
from math import radians


def c(rx, ry, rz):
    A_CSS_P_trans = hom_translation_matrix(
        tx=0.265, ty=0, tz=0.014)
    A_CSS_P_rot = hom_rotation(x_axis_rotation_matrix(rx) @ 
                               y_axis_rotation_matrix(ry) @ 
                               z_axis_rotation_matrix(rz))
    A_P_SPH1_2 = hom_translation_matrix(
        tx=0.015, ty=0.029, tz=-0.0965)
    A_P_SPH2_2 = hom_translation_matrix(
        tx=0.015, ty=-0.029, tz=-0.0965)

    A_CSS_P = A_CSS_P_trans @ A_CSS_P_rot
    A_c1    = A_CSS_P @ A_P_SPH1_2
    A_c2    = A_CSS_P @ A_P_SPH2_2

    return get_translation(A_c1),get_translation(A_c2)


def p1(theta):
    A_CCS_lsm_tran = hom_translation_matrix(
        tx=0.139807669447128, ty=0.0549998406976098, tz=-0.051)
    A_CCS_lsm_rot  = hom_rotation(z_axis_rotation_matrix(radians(-338.5255)))  
    A_MCS1_JOINT   = hom_rotation(z_axis_rotation_matrix(theta))
    A_MCS1_SP11    = hom_translation_matrix(
        tx=0.085, ty=0, tz=-0.0245)

    A_CCS_SP11 =A_CCS_lsm_tran @ A_CCS_lsm_rot @ A_MCS1_JOINT @ A_MCS1_SP11
    return get_translation(A_CCS_SP11)


def p2(theta):
    A_CCS_rsm_tran = hom_translation_matrix(
        tx=0.139807669447128, ty=-0.0549998406976098, tz=-0.051)
    A_CCS_rsm_rot  = hom_rotation(z_axis_rotation_matrix(radians(-21.4745))) 
    A_MCS2_JOINT   = hom_rotation(z_axis_rotation_matrix(theta))
    A_MCS2_SP21    = hom_translation_matrix(
        tx=0.085, ty=0, tz=-0.0245)

    A_CSS_SP21 =  A_CCS_rsm_tran @ A_CCS_rsm_rot @ A_MCS2_JOINT @ A_MCS2_SP21
    return get_translation(A_CSS_SP21)


def swing_to_gimbal(state: Dict[str, float], tips: Dict[str, float] = None):
    x_0 = [0,0,0]
    if tips:
        x_0[2] = tips['rx']
        x_0[3] = tips['ry']
        x_0[4] = tips['rz']

    nlp        = {'x':virtual_state ,'f':closing_equation,'p':actuated_state}
    nlp_solver = nlpsol('swing_to_gimbal','ipopt',nlp,opts)
    solution   = nlp_solver(x0 = x_0,
                            p  = [state['swing_left'],state['swing_right']])
    sol_vector = np.array(solution['x'])
    return {'gimbal_joint': {'rx': sol_vector[0][0], 'ry': sol_vector[1][0], 'rz': sol_vector[2][0]}}


def gimbal_to_swing(state: Dict[str,Dict[str, float]], tips: Dict[str, float] = None):
    x_0 = [0,0]
    if tips:
        x_0[0] = tips['swing_left'] 
        x_0[1] = tips['swing_right']
    
    nlp        = {'x':actuated_state ,'f':closing_equation,'p':virtual_state}
    nlp_solver = nlpsol('gimbal_to_swing','ipopt',nlp,opts)
    solution   = nlp_solver(x0 = x_0, 
                            p  = [state['gimbal_joint']['rx'], state['gimbal_joint']['ry'],state['gimbal_joint']['rz']])
    sol_vector = np.array(solution['x'])
    return {'swing_left': sol_vector[0][0], 'swing_right': sol_vector[1][0]}


theta_left  = SX.sym('theta_left')
theta_right = SX.sym('theta_right')
gimbal_x    = SX.sym('gimbal_x')
gimbal_y    = SX.sym('gimbal_y')
gimbal_z    = SX.sym('gimbal_z')

virtual_state  = vertcat(gimbal_x ,gimbal_y ,gimbal_z )
actuated_state = vertcat(theta_left,theta_right)

opts             = {'ipopt.print_level':0, 'print_time':0}
r                = 0.11
c1, c2           = c(rx=gimbal_x, ry=gimbal_y, rz=gimbal_z)
closing_equation = ((c1-p1(theta_right)).T @ (c1-p1(theta_right)) -r**2)**2+(
                    (c2-p2(theta_left)).T @ (c2-p2(theta_left)) -  r**2)**2



A_CSS_P_trans = Transformation(name   = 'A_CSS_P_trans',
                                values = {'tx': 0.265, 'tz': 0.014})
A_CSS_P_rot   = Transformation(name   = 'gimbal_joint',
                                values = {'rx': 0, 'ry': 0, 'rz': 0}, 
                                state_variables = ['rx', 'ry', 'rz'])

closed_chain  = KinematicGroup(name                    = 'closed_chain', 
                                virtual_transformations = [A_CSS_P_trans,A_CSS_P_rot], 
                                actuated_state          = {'swing_left': 0,'swing_right': 0}, 
                                actuated_to_virtual     = swing_to_gimbal, 
                                virtual_to_actuated     = gimbal_to_swing)

A_P_LL             = Transformation(name   = 'A_P_LL', 
                                    values = {'tx': 1.640, 'tz': -0.037, })
A_LL_LL_zero       = Transformation(name   = 'zero_angle_convention',
                                    values = {'ry': radians(-3)})
A_LL_zero_LL_joint = Transformation(name   = 'extend_joint',
                                    values = {'ry': 0}, 
                                    state_variables = ['ry'])
A_LL_Joint_FCS     = Transformation(name   = 'A_LL_Joint_FCS', 
                                    values = {'tx': -1.5})


triped_leg     = Robot([closed_chain,A_P_LL,A_LL_LL_zero,A_LL_zero_LL_joint,A_LL_Joint_FCS])
triped_leg.set_actuated_state({'swing_left': 0, 'swing_right': 0})

 