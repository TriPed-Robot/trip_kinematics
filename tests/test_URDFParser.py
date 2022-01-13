import unittest
import os
import xml.etree.ElementTree as ET

import kinpy as kp
import numpy as np
from trip_kinematics import Utility
import trip_kinematics.URDFParser
import trip_kinematics.Robot

urdf_examples_dir = os.path.join('tests', 'urdf_examples')


def to_kp(state):
    return {
        "_".join(joint_name.split("_")[:-2]): value
        for joint_name, value in state.items()
    }


def to_trip(state):
    return state


def initialize_state(robot):
    return {
        joint_name: 0
        for joint_name in robot.get_actuated_state()
    }


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


def compare_urdf_trip_vs_kinpy(path):
    path = os.path.join(urdf_examples_dir, path)

    # Setup TriP robot using the URDF parser
    try:
        robot = trip_kinematics.Robot(trip_kinematics.URDFParser.from_urdf(path))
        state_init = initialize_state(robot)
        robot.set_actuated_state(to_trip(state_init))
    except ValueError as err:
        raise ValueError(f'File {path} contains unupported joint type ({err})') from err

    # Setup kinpy chain
    try:
        chain_kinpy = create_kinpy_chain(path)
        chain_kinpy.forward_kinematics(to_kp(state_init))
    except KeyError as err:
        raise ValueError(f'File {path} contains unupported joint type ({err})') from err

    for joint, joint_dict in get_joint_tree_dict(path).items():
        # Calculate homogenous transformation matrix of kinpy forward kinematics
        transf_kp = chain_kinpy.forward_kinematics(to_kp(state_init))[joint_dict['child_link']]

        kp_rot_matrix = Utility.quat_rotation_matrix(*transf_kp.rot).astype('float64')
        transf_kp_hom_rot = Utility.hom_rotation(kp_rot_matrix).astype('float64')
        transf_kp_hom_pos = Utility.hom_translation_matrix(*transf_kp.pos).astype('float64')

        transf_kp_hom = transf_kp_hom_pos @ transf_kp_hom_rot

        # Calculate homogenous matrix of TriP forward kinematic for every joint
        transf_trip_hom = trip_kinematics.forward_kinematics(robot, joint).astype('float64')

        # Compare the two matrices and ensure they are close within reason
        assert np.allclose(transf_kp_hom, transf_trip_hom)


class TestStates(unittest.TestCase):
    # test movement not aligned on single axis <- this is already done in other
    # tests, skipping for now

    def test_all_urdf_files(self):
        # Test of single fixed joint
        compare_urdf_trip_vs_kinpy("one_fixed_joint.urdf")

        # Test of single continuous joint
        compare_urdf_trip_vs_kinpy("one_continuous_joint.urdf")

        # Test of single revolute joint
        compare_urdf_trip_vs_kinpy("one_revolute_joint.urdf")

        # Test of single prismatic joint
        compare_urdf_trip_vs_kinpy("one_prismatic_joint.urdf")

        # Test of fixed and revolute joints combined
        compare_urdf_trip_vs_kinpy("fixed_to_revolute_joint.urdf")

        # Test of prismatic and continuous joints combined
        compare_urdf_trip_vs_kinpy("prismatic_to_continuous_joint.urdf")

        # Test of large tree
        compare_urdf_trip_vs_kinpy("large_tree_test.urdf")

        # Test of multiple connections to "ground"
        compare_urdf_trip_vs_kinpy("test.urdf")

    def test_align_vectors(self):
        test_cases = [
            ([1, 2, 3], [4, 5, 6]),     # Random angle
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
