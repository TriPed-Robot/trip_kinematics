import unittest
import os
import xml.etree.ElementTree as ET
import random

import kinpy as kp
import numpy as np
from trip_kinematics import Utility, URDFParser, Robot
import trip_kinematics


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
    return URDFParser._build_joint_tree_dict(joints)


def compare_urdf_trip_vs_kinpy(path, rng_states_count=10):
    # Setup TriP robot using the URDF parser
    try:
        robot = Robot(URDFParser.from_urdf(path))
        state_init = initialize_state(robot)
    except ValueError as err:
        raise ValueError(f'File {path} contains unsupported joint type ({err})') from err

    # Setup kinpy chain
    try:
        chain_kinpy = create_kinpy_chain(path)
    except KeyError as err:
        raise ValueError(f'File {path} contains unsupported joint type ({err})') from err

    test_states = [state_init]

    for seed in range(rng_states_count):
        random.seed(seed)
        new_state = {
            joint: random.uniform(-np.pi, np.pi)
            for joint in state_init.keys()
        }
        test_states.append(new_state)

    for state in test_states:
        robot.set_actuated_state(to_trip(state))
        chain_kinpy.forward_kinematics(to_kp(state))

        # Calculate homogenous transformation matrices using both kinpy and TriP for every joint
        # and compare them. joint_dict contains info about connections to other joints.
        for joint, joint_dict in get_joint_tree_dict(path).items():
            # kinpy
            transf_kp = chain_kinpy.forward_kinematics(to_kp(state))[joint_dict['child_link']]
            kp_rot_matrix = Utility.quat_rotation_matrix(*transf_kp.rot).astype('float64')
            transf_kp_hom_rot = Utility.hom_rotation(kp_rot_matrix).astype('float64')
            transf_kp_hom_pos = Utility.hom_translation_matrix(*transf_kp.pos).astype('float64')
            transf_kp_hom = transf_kp_hom_pos @ transf_kp_hom_rot

            # TriP
            transf_trip_hom = trip_kinematics.forward_kinematics(robot, joint).astype('float64')

            assert np.allclose(transf_kp_hom, transf_trip_hom)


class TestStates(unittest.TestCase):
    def test_all_urdf_files(self):
        paths = [
            "one_fixed_joint",                  # Single fixed joint
            "one_continuous_joint",             # Single continuous joint
            "one_revolute_joint",               # Single revolute joint
            "one_prismatic_joint",              # Single prismatic joint
            "fixed_to_revolute_joint",          # Fixed and revolute joints combined
            "prismatic_to_continuous_joint",    # Prismatic and continuous joints combined
            "large_tree_test",                  # Large tree
            "ground_to_many",                   # Multiple connections to "ground"
        ]

        urdf_examples_dir = os.path.join('tests', 'urdf_examples')

        for path in paths:
            full_path = os.path.join(urdf_examples_dir, path + '.urdf')
            print(full_path)
            compare_urdf_trip_vs_kinpy(full_path)

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

        for target, to_align in test_cases:
            target /= np.linalg.norm(target)
            to_align /= np.linalg.norm(to_align)

            rotation_matrix = URDFParser.align_vectors(target, to_align)
            aligned = to_align @ rotation_matrix

            assert np.all(np.isclose(aligned, target))
