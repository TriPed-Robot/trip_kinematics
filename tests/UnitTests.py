import unittest
from trip_kinematics.Robot import Robot, forward_kinematics, inverse_kinematics
from trip_kinematics.KinematicGroup import KinematicGroup, Transformation
from math import radians, degrees
from copy import deepcopy

# Simple scara robot

arm_1 = Transformation(name="arm_1", values={'tx': 1})
joint = Transformation(name="joint", values={'rz': 0}, state_variables=['rz'])
arm_2 = Transformation(name="arm_2", values={'tx': 1})


def f_mapping(state):
    rot = state[0]['z']
    return [{}, {'rz': rot}, {}]


def g_mapping(state):
    rot = state[1]['rz']
    return[{'z': rot}]


class TestStates(unittest.TestCase):

    def test_set_actuated_state(self):
        scara_arm = KinematicGroup(name="scara", virtual_transformations=[
            arm_1, joint, arm_2], actuated_state=[{'z': 0}], f_mapping=f_mapping, g_mapping=g_mapping)

        scara = Robot([scara_arm])

        inital_position = forward_kinematics(scara)
        scara_arm.set_actuated_state([{'z': radians(90)}])
        end_position = forward_kinematics(scara)

        self.assertEqual(str(list(inital_position)), str([2.0, 0.0, 0.0]))
        self.assertEqual(str(list(end_position)), str([1.0, 1.0, 0.0]))

    def test_trivial_f_mapping(self):
        scara_arm = KinematicGroup(name="scara", virtual_transformations=[
            arm_1, joint, arm_2])
        scara = Robot([scara_arm])
        inital_position = forward_kinematics(scara)
        scara_arm.set_actuated_state([{'joint_rz': radians(90)}])
        end_position = forward_kinematics(scara)

        self.assertEqual(str(list(inital_position)), str([2.0, 0.0, 0.0]))
        self.assertEqual(str(list(end_position)), str([1.0, 1.0, 0.0]))

    def test_set_virtual_state(self):
        scara_arm = KinematicGroup(name="scara", virtual_transformations=[
            arm_1, joint, arm_2], actuated_state=[{'z': 0}], f_mapping=f_mapping, g_mapping=g_mapping)

        scara = Robot([scara_arm])
        inital_angle = inverse_kinematics(scara, [2.0, 0.0, 0.0])
        scara_arm.set_virtual_state([{}, {'rz': radians(90)}, {}])
        end_angle = inverse_kinematics(scara, [1.0, 1.0, 0.0])

        self.assertEqual(str(inital_angle), str([[[{'z': 0.0}]]]))
        self.assertEqual(str(end_angle), str([[[{'z': radians(90)}]]]))

    def test_trivial_g_mapping(self):
        scara_arm = KinematicGroup(name="scara", virtual_transformations=[
            arm_1, joint, arm_2])
        scara = Robot([scara_arm])
        inital_angle = inverse_kinematics(scara, [2.0, 0.0, 0.0])
        scara_arm.set_virtual_state(
            [{}, {'rz': radians(90)}, {'rz': radians(90)}, {}])
        end_angle = inverse_kinematics(scara, [1.0, 1.0, 0.0])

        self.assertEqual(str(inital_angle), str([[[{'joint_rz': 0.0}]]]))
        self.assertEqual(str(end_angle), str([[[{'joint_rz': radians(90)}]]]))


if __name__ == '__main__':
    unittest.main()
