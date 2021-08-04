from trip_kinematics.KinematicGroup import KinematicGroup, Transformation
from trip_kinematics.Robot import Robot, inverse_kinematics, forward_kinematics
from casadi import Opti
from typing import Dict, List
from trip_kinematics.HomogenTransformationMatrix import HomogenousTransformationMatrix
import numpy as np
from math import radians, sin, cos


def c(rx, ry, rz, opti):
    A_CSS_P_trans = HomogenousTransformationMatrix(
        tx=0.265, ty=0, tz=0.014)

    A_CSS_P_rot = HomogenousTransformationMatrix(
        conv='xyz', rx=rx, ry=ry, rz=rz)

    A_CSS_P = A_CSS_P_trans * A_CSS_P_rot

    T_P_SPH1_2 = np.array([-0.015, -0.029, 0.0965]) * -1
    T_P_SPH2_2 = np.array([-0.015, 0.029, 0.0965]) * -1
    x0, y0, z0 = T_P_SPH1_2
    x1, y1, z1 = T_P_SPH2_2

    A_P_SPH1_2 = HomogenousTransformationMatrix(
        tx=x0, ty=y0, tz=z0, conv='xyz')
    A_P_SPH2_2 = HomogenousTransformationMatrix(
        tx=x1, ty=y1, tz=z1, conv='xyz')

    A_c1 = A_CSS_P * A_P_SPH1_2
    A_c2 = A_CSS_P * A_P_SPH2_2
    # print(A_c1)
    c1 = A_c1.get_translation()
    c2 = A_c2.get_translation()

    c1_mx = opti.variable(3, 1)
    c1_mx[0, 0] = c1[0]
    c1_mx[1, 0] = c1[1]
    c1_mx[2, 0] = c1[2]

    c2_mx = opti.variable(3, 1)
    c2_mx[0, 0] = c2[0]
    c2_mx[1, 0] = c2[1]
    c2_mx[2, 0] = c2[2]

    c1 = c1_mx
    c2 = c2_mx
    return c1, c2


def p1(theta, opti):
    A_CCS_lsm_tran = HomogenousTransformationMatrix(
        tx=0.139807669447128, ty=0.0549998406976098, tz=-0.051)

    A_CCS_lsm_rot = HomogenousTransformationMatrix(
        rz=radians(-338.5255), conv='xyz')  # radians()34.875251275010434

    A_CCS_lsm = A_CCS_lsm_tran * A_CCS_lsm_rot

    A_MCS1_JOINT = HomogenousTransformationMatrix(
        rz=theta, conv='xyz')

    A_CSS_MCS1 = A_CCS_lsm * A_MCS1_JOINT

    A_MCS1_SP11 = HomogenousTransformationMatrix(
        tx=0.085, ty=0, tz=-0.0245)

    A_CCS_SP11 = A_CSS_MCS1 * A_MCS1_SP11

    p1 = A_CCS_SP11.get_translation()
    p1_mx = opti.variable(3, 1)
    p1_mx[0, 0] = p1[0]
    p1_mx[1, 0] = p1[1]
    p1_mx[2, 0] = p1[2]
    return p1_mx


def p2(theta, opti):
    A_CCS_rsm_tran = HomogenousTransformationMatrix(
        tx=0.139807669447128, ty=-0.0549998406976098, tz=-0.051)

    A_CCS_rsm_rot = HomogenousTransformationMatrix(
        rz=radians(-21.4745), conv='xyz')  # radians(-21.4745)-34.875251275010434

    A_CCS_rsm = A_CCS_rsm_tran*A_CCS_rsm_rot

    A_MCS2_JOINT = HomogenousTransformationMatrix(
        rz=theta, conv='xyz')

    A_CSS_MCS2 = A_CCS_rsm * A_MCS2_JOINT

    A_MCS2_SP21 = HomogenousTransformationMatrix(
        tx=0.085, ty=0, tz=-0.0245)

    A_CSS_SP21 = A_CSS_MCS2 * A_MCS2_SP21

    p2 = A_CSS_SP21.get_translation()
    p2_mx = opti.variable(3, 1)
    p2_mx[0, 0] = p2[0]
    p2_mx[1, 0] = p2[1]
    p2_mx[2, 0] = p2[2]
    return p2_mx


def mapping_f(state: List[Dict[str, float]], tips: Dict[str, float] = None):

    opti = Opti()
    r = 0.11

    theta_left = state[0]['t1']
    theta_right = state[0]['t2']

    gimbal_x = opti.variable()
    gimbal_y = opti.variable()
    gimbal_z = opti.variable()

    if tips:
        opti.set_initial(gimbal_x, tips['rx'])
        opti.set_initial(gimbal_y, tips['ry'])
        opti.set_initial(gimbal_z, tips['rz'])

    c1, c2 = c(rx=gimbal_x, ry=gimbal_y, rz=gimbal_z, opti=opti)
    closing_equation = ((c1-p1(theta_right, opti)).T @ (c1-p1(theta_right, opti)) -
                        r**2)**2+((c2-p2(theta_left, opti)).T @ (c2-p2(theta_left, opti)) - r**2)**2
    # print(p1(theta_left, opti))
    opti.minimize(closing_equation)
    p_opts = {"print_time": False}
    s_opts = {"print_level": 0, "print_timing_statistics": "no"}
    opti.solver('ipopt', p_opts, s_opts)
    sol = opti.solve()
    return [{}, {'rx': sol.value(gimbal_x), 'ry': sol.value(gimbal_y), 'rz': sol.value(gimbal_z)}]


def mapping_g(state: List[Dict[str, float]], tips: Dict[str, float] = None):

    opti = Opti()
    r = 0.11

    theta_left = opti.variable()
    theta_right = opti.variable()

    if tips:
        opti.set_initial(theta_left, tips['t1'])
        opti.set_initial(theta_right, tips['t2'])

    gimbal_x = state[1]['rx']
    gimbal_y = state[1]['ry']
    gimbal_z = state[1]['rz']
    c1, c2 = c(rx=gimbal_x, ry=gimbal_y, rz=gimbal_z, opti=opti)
    closing_equation = ((c1-p1(theta_right, opti)).T @ (c1-p1(theta_right, opti)) -
                        r**2)**2+((c2-p2(theta_left, opti)).T @ (c2-p2(theta_left, opti)) - r**2)**2
    opti.minimize(closing_equation)
    p_opts = {"print_time": False}
    s_opts = {"print_level": 0, "print_timing_statistics": "no"}
    opti.solver('ipopt', p_opts, s_opts)
    sol = opti.solve()
    return [{'t1': sol.value(theta_left), 't2': sol.value(theta_right)}]


A_CSS_P_trans = Transformation(name='A_CSS_P_trans',
                               values={'tx': 0.265, 'tz': 0.014})

A_CSS_P_rot = Transformation(name='A_CSS_P_rot',
                             values={'rx': 0, 'ry': 0, 'rz': 0}, state_variables=['rx', 'ry', 'rz'])

gimbal_joint = KinematicGroup(name='gimbal_joint', virtual_transformations=[A_CSS_P_trans,
                                                                            A_CSS_P_rot], actuated_state=[{'t1': 0, 't2': 0}], f_mapping=mapping_f, g_mapping=mapping_g)

A_P_LL = Transformation(name='A_P_LL', values={'tx': 1.640, 'tz': -0.037, })

zero_angle_convention = Transformation(name='zero_angle_convention',
                                       values={'ry': radians(3)})

LL_revolute_joint = Transformation(name='LL_revolute_joint',
                                   values={'ry': 0}, state_variables=['ry'])

A_LL_Joint_FCS = Transformation(name='A_LL_Joint_FCS', values={'tx': -1.5})

extend_motor = KinematicGroup(name='extend_motor', virtual_transformations=[A_P_LL, zero_angle_convention,
                                                                            LL_revolute_joint, A_LL_Joint_FCS], parent=gimbal_joint)

triped_leg = Robot([gimbal_joint, extend_motor])

gimbal_joint.set_actuated_state([{'t1': 0, 't2': 0}])
