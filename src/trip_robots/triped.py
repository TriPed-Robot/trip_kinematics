from copy import deepcopy
from typing import Dict
from math import radians

from trip_kinematics.KinematicGroup import KinematicGroup, Transformation
from trip_kinematics.Robot import Robot
from trip_robots.triped_leg import swing_to_gimbal, gimbal_to_swing


def leg_model(leg_number: str):
    """Helper function that constructs each TriPed leg as a list of Transformations.

    Args:
        leg_number (str): The leg number which determins the orientation.
                          Acceptable values [0,1,2]
    """
    def rename_swing_to_gimbal(swing: Dict[str, float], tips: Dict[str, float] = None):
        swing = deepcopy(swing)
        swing['swing_left'] = swing[leg_name+'swing_left']
        swing['swing_right'] = swing[leg_name+'swing_right']
        del swing[leg_name+'swing_left']
        del swing[leg_name+'swing_right']

        if tips is not None:
            tips = deepcopy(tips)
            tips['gimbal_joint'] = tips[leg_name+'gimbal_joint']
            del tips[leg_name+'gimbal_joint']

        gimbal = swing_to_gimbal(swing, tips)

        gimbal[leg_name+'gimbal_joint'] = gimbal['gimbal_joint']
        del gimbal['gimbal_joint']

        return gimbal

    def rename_gimbal_to_swing(gimbal: Dict[str, float], tips: Dict[str, float] = None):
        gimbal = deepcopy(gimbal)
        gimbal['gimbal_joint'] = gimbal[leg_name+'gimbal_joint']
        del gimbal[leg_name+'gimbal_joint']

        if tips is not None:
            tips = deepcopy(tips)
            tips['swing_left'] = tips[leg_name+'swing_left']
            tips['swing_right'] = tips[leg_name+'swing_right']
            del tips[leg_name+'swing_left']
            del tips[leg_name+'swing_right']

        swing = gimbal_to_swing(gimbal, tips)

        swing[leg_name+'swing_left'] = swing['swing_left']
        swing[leg_name+'swing_right'] = swing['swing_right']
        del swing['swing_left']
        del swing['swing_right']

        return swing

    leg_name = 'leg_'+str(leg_number)+'_'

    leg_rotation = Transformation(name=leg_name+'leg_rotation',
                                  values={'rz': -1*radians(120)*leg_number})
    a_ccs_p_trans = Transformation(name=leg_name+'A_ccs_P_trans',
                                   values={'tx': 0.265, 'tz': 0.014},
                                   parent=leg_rotation)
    a_ccs_p_rot = Transformation(name=leg_name+'gimbal_joint',
                                 values={'rx': 0, 'ry': 0, 'rz': 0},
                                 state_variables=['rx', 'ry', 'rz'],
                                 parent=a_ccs_p_trans)

    closed_chain = KinematicGroup(name=leg_name+'closed_chain',
                                  virtual_chain=[leg_rotation,
                                                 a_ccs_p_trans, a_ccs_p_rot],
                                  actuated_state={
                                      leg_name+'swing_left': 0, leg_name+'swing_right': 0},
                                  actuated_to_virtual=rename_swing_to_gimbal,
                                  virtual_to_actuated=rename_gimbal_to_swing)

    a_p_ll = Transformation(name=leg_name+'A_P_LL',
                            values={'tx': 1.640, 'tz': -0.037, },
                            parent=closed_chain)
    a_ll_zero = Transformation(name=leg_name+'zero_angle_convention',
                               values={'ry': radians(-3)},
                               parent=a_p_ll)
    a_ll_zero_ll_joint = Transformation(name=leg_name+'extend_joint',
                                        values={'ry': 0},
                                        state_variables=['ry'],
                                        parent=a_ll_zero)
    a_ll_joint_fcs = Transformation(name=leg_name+'A_LL_Joint_FCS',
                                    values={'tx': -1.5},
                                    parent=a_ll_zero_ll_joint)

    return [closed_chain, a_ll_zero_ll_joint, a_ll_joint_fcs, a_p_ll, a_ll_zero]


triped = Robot(leg_model(0)+leg_model(1)+leg_model(2))
triped.set_actuated_state({'leg_0_swing_left': 0, 'leg_0_swing_right': 0})
