import unittest
import os
import xml.etree.ElementTree as ET

import kinpy as kp
import numpy as np
# from trip_kinematics.URDFParser import align_vectors
import trip_kinematics.URDFParser
import trip_kinematics.Robot

urdf_examples_dir = os.path.join('tests', 'urdf_examples')


def state_to_kinpy(state):
    return {
        "_".join(joint_name.split("_")[:-2]): value
        for joint_name, value in state.items()
    }


def state_to_trip(state):
    return state


def get_state(robot):
    return {
        joint_name: 0
        for joint_name in robot.get_actuated_state()
    }


def create_trip_robot(path):
    chain_trip = trip_kinematics.URDFParser.from_urdf(path)
    return trip_kinematics.Robot(chain_trip)


def create_kinpy_chain(path):
    with open(path, encoding='utf8') as file:
        urdf_data_str = file.read()
        return kp.build_chain_from_urdf(urdf_data_str)


def get_joint_tree_dict(path):
    tree = ET.parse(path)
    root = tree.getroot()
    joints = root.findall('joint')

    # pylint: disable=protected-access
    return trip_kinematics.URDFParser._build_joint_tree_dict(joints)


def run_test(path):
    # setup TriP robot using the URDF parser
    # this needs a try catch
    path = os.path.join(urdf_examples_dir, path)
    robot = create_trip_robot(path)
    state = get_state(robot)

    # setup kinpy chain
    # this needs a try catch
    chain_kinpy = create_kinpy_chain(path)
    chain_kinpy.forward_kinematics(state_to_kinpy(state))

    # this needs a try catch?
    robot.set_actuated_state(state_to_trip(state))

    joint_tree_dict = get_joint_tree_dict(path)

    for joint, joint_dict in joint_tree_dict.items():
        # calculate homogenous matrix of kinpy forward kinematic for every joint
        transf_kp = chain_kinpy.forward_kinematics(state_to_kinpy(state))[joint_dict['child_link']]

        transf_kp_hom_pos = \
            trip_kinematics.Utility.hom_translation_matrix(*transf_kp.pos) \
            .astype('float64')
        transf_kp_hom_rot = \
            trip_kinematics.Utility.hom_rotation(
                trip_kinematics.Utility.quat_rotation_matrix(*transf_kp.rot).astype('float64')
            ) \
            .astype('float64')

        transf_kp_hom = transf_kp_hom_pos @ transf_kp_hom_rot

        # calculate homogenous matrix of TriP forward kinematic for every joint
        transf_trip_hom = \
            trip_kinematics.forward_kinematics(robot, joint).astype('float64')

        # compare the two matrices and ensure they are close within reason
        if not np.allclose(transf_kp_hom, transf_trip_hom):
            return False
    return True


class TestStates(unittest.TestCase):
    # test movement not aligned on single axis <- this is already done in other
    # tests, skipping for now

    def test_all_urdf_files(self):
        # test of single fixed joint
        assert run_test("one_fixed_joint.urdf")

        # test of single continuous joint
        assert run_test("one_continuous_joint.urdf")

        # test of single revolute joint
        assert run_test("one_revolute_joint.urdf")

        # test of single prismatic joint
        assert run_test("one_prismatic_joint.urdf")

        # test of fixed and revolute joints combined
        assert run_test("fixed_to_revolute_joint.urdf")

        # test of prismatic and continuous joints combined
        assert run_test("prismatic_to_continuous_joint.urdf")

        # test of large tree
        assert run_test("large_tree_test.urdf")

        # test of multiple connections to "ground"
        assert run_test("test.urdf")

    def test_align_vectors(self):
        test_cases = [
            ([1, 2, 3], [4, 5, 6]),     # random angle
            ([1, 0, 0], [0, 1, 0]),     # 90 degrees
            ([1, 0, 0], [-1, 0, 0]),    # -180 degrees
            ([0, 0, 1], [0, 0, -1]),    # -180 degrees
            ([0, 1, 4], [0, -1, -4]),   # -180 degrees
            ([-3, 1, 4], [3, -1, -4]),  # -180 degrees
            ([8, 9, 3], [8, 9, 3]),     # zero angle
        ]

        for case in test_cases:
            target, to_align = np.array(case[0]), np.array(case[1])
            target = target / np.linalg.norm(target)
            to_align = to_align / np.linalg.norm(to_align)
            rotation_matrix = trip_kinematics.URDFParser.align_vectors(target, to_align)
            aligned = to_align @ rotation_matrix
            assert np.all(np.isclose(aligned, target))
