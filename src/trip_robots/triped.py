from copy import deepcopy
from typing import Dict
from math import radians

from trip_kinematics.KinematicGroup import KinematicGroup, Transformation
from trip_kinematics.Robot import Robot
from trip_robots.triped_leg import *


def leg_model(leg_number: str):
    def rename_swing_to_gimbal(swing: Dict[str, float], tips: Dict[str, float] = None):
        swing = deepcopy(swing)
        swing['swing_left'] = swing[leg_name+'swing_left']
        swing['swing_right'] = swing[leg_name+'swing_right']
        del swing[leg_name+'swing_left']
        del swing[leg_name+'swing_right']

        gimbal = swing_to_gimbal(swing, tips)

        gimbal[leg_name+'gimbal_joint'] = gimbal['gimbal_joint']
        del gimbal['gimbal_joint']

        return gimbal

    def rename_gimbal_to_swing(gimbal: Dict[str, float], tips: Dict[str, float] = None):
        gimbal = deepcopy(gimbal)
        gimbal['gimbal_joint'] = gimbal[leg_name+'gimbal_joint']
        del gimbal[leg_name+'gimbal_joint']

        swing = gimbal_to_swing(gimbal, tips)

        swing[leg_name+'swing_left'] = swing['swing_left']
        swing[leg_name+'swing_right'] = swing['swing_right']
        del swing['swing_left']
        del swing['swing_right']

        return swing

    leg_name = 'leg'+str(leg_number)+'_'

    leg_rotation = Transformation(name=leg_name+'leg_rotation',
                                  values={'rz': -1*radians(120)*leg_number})
    A_CSS_P_trans = Transformation(name=leg_name+'A_CSS_P_trans',
                                   values={'tx': 0.265, 'tz': 0.014},
                                   parent=leg_rotation)
    A_CSS_P_rot = Transformation(name=leg_name+'gimbal_joint',
                                 values={'rx': 0, 'ry': 0, 'rz': 0},
                                 state_variables=['rx', 'ry', 'rz'],
                                 parent=A_CSS_P_trans)

    closed_chain = KinematicGroup(name=leg_name+'closed_chain',
                                  virtual_chain=[leg_rotation,
                                                 A_CSS_P_trans, A_CSS_P_rot],
                                  actuated_state={
                                      leg_name+'swing_left': 0, leg_name+'swing_right': 0},
                                  actuated_to_virtual=rename_swing_to_gimbal,
                                  virtual_to_actuated=rename_gimbal_to_swing)

    A_P_LL = Transformation(name=leg_name+'A_P_LL',
                            values={'tx': 1.640, 'tz': -0.037, },
                            parent=closed_chain)
    A_LL_LL_zero = Transformation(name=leg_name+'zero_angle_convention',
                                  values={'ry': radians(-3)},
                                  parent=A_P_LL)
    A_LL_zero_LL_joint = Transformation(name=leg_name+'extend_joint',
                                        values={'ry': 0},
                                        state_variables=['ry'],
                                        parent=A_LL_LL_zero)
    A_LL_Joint_FCS = Transformation(name=leg_name+'A_LL_Joint_FCS',
                                    values={'tx': -1.5},
                                    parent=A_LL_zero_LL_joint)

    return [closed_chain, A_LL_zero_LL_joint, A_LL_Joint_FCS, A_P_LL, A_LL_LL_zero]


triped = Robot(leg_model(0)+leg_model(1)+leg_model(2))
triped.set_actuated_state({'leg0_swing_left': 0, 'leg0_swing_right': 0})
