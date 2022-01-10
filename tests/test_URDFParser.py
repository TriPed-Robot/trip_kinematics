import unittest
import os
import xml.etree.ElementTree as ET

import kinpy as kp
import numpy as np
# from trip_kinematics.URDFParser import align_vectors
import trip_kinematics.URDFParser
import trip_kinematics.Robot

def state_to_kinpy(state):
    return {
        "_".join(joint_name.split("_")[:-2]) : value
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
    return trip_kinematics.URDFParser._build_joint_tree_dict(joints)


def run_test(path):
    # setup TriP robot using the URDF parser
    # this needs a try catch
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
        transf_kp = chain_kinpy.forward_kinematics(state_to_kinpy(state)) \
            [joint_dict['child_link']]

        transf_kp_hom_pos = \
            trip_kinematics.Utility.hom_translation_matrix(*transf_kp.pos) \
            .astype('float64')
        transf_kp_hom_rot = \
            trip_kinematics.Utility.hom_rotation(trip_kinematics.Utility \
            .quat_rotation_matrix(*transf_kp.rot).astype('float64')) \
            .astype('float64')

        transf_kp_hom = transf_kp_hom_pos @ transf_kp_hom_rot

        # calculate homogenous matrix of TriP forward kinematic for every joint
        transf_trip_hom = \
            trip_kinematics.forward_kinematics(robot, joint).astype('float64')

        # compare the two matrices and ensure they are close within reason
        assert np.allclose(transf_kp_hom, transf_trip_hom)

class TestStates(unittest.TestCase):

    # test of single fixed joint

    # test of single continuous joint

    # test of single revolute joint

    # test of single prismatic joint

    # test of fixed and revolute joints combined

    # test of prismatic and continuous joints combined

    # test of large tree
    def test_large_tree_urdf(self):
        urdf_examples_dir = os.path.join('tests', 'urdf_examples')
        urdf_examples_filenames = os.listdir(urdf_examples_dir)
        full_path = os.path.join(urdf_examples_dir, "large_tree_test.urdf")
        run_test(full_path)

    # test of multiple connections to "ground"

    # test movement not aligned on single axis

    def test_all_urdf_files(self):

        urdf_examples_dir = os.path.join('tests', 'urdf_examples')
        urdf_examples_filenames = os.listdir(urdf_examples_dir)

        for filename in urdf_examples_filenames:
            full_path = os.path.join(urdf_examples_dir, filename)
            run_test(full_path)

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
            a, b = np.array(case[0]), np.array(case[1])
            a = a / np.linalg.norm(a)
            b = b / np.linalg.norm(b)
            rotation_matrix = trip_kinematics.URDFParser.align_vectors(a, b)
            aligned = b @ rotation_matrix
            assert np.all(np.isclose(aligned, a))


if __name__ == '__main__':
    unittest.main()
