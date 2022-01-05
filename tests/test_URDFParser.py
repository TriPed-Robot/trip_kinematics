import unittest
import os
import xml.etree.ElementTree as ET

import kinpy as kp
import numpy as np
# from trip_kinematics.URDFParser import align_vectors
import trip_kinematics.URDFParser
import trip_kinematics.Robot


def state_to_kinpy(state):
    return state


def state_to_trip(state):
    return {
        joint_name + '_mov_rz': value
        for joint_name, value in state.items()
    }


def printy(i, robot):
    np.set_printoptions(precision=3, suppress=True)
    print(robot.get_endeffectors()[i], '\n\n', trip_kinematics.forward_kinematics(
        robot, robot.get_endeffectors()[i]).astype('float64'))


class TestStates(unittest.TestCase):
    def test_simple_inverse_kinematics(self):
        # TODO remove later
        np.set_printoptions(precision=3, suppress=True)

        urdf_examples_dir = os.path.join('tests', 'urdf_examples')
        urdf_examples_filenames = os.listdir(urdf_examples_dir)

        for filename in urdf_examples_filenames:
            full_path = os.path.join(urdf_examples_dir, filename)

            with open(full_path, encoding='utf8') as file:
                try:
                    # setup kinpy chain
                    urdf_data_str = file.read()
                    chain_kinpy = kp.build_chain_from_urdf(urdf_data_str)
                    state = {
                        joint_name: 0
                        for joint_name in chain_kinpy.get_joint_parameter_names()
                    }
                    chain_kinpy.forward_kinematics(state_to_kinpy(state))

                    # setup TriP robot using the URDF parser
                    chain_trip = trip_kinematics.URDFParser.from_urdf(full_path)
                    robot = trip_kinematics.Robot(chain_trip)
                    robot.set_actuated_state(state_to_trip(state))


                    tree = ET.parse(full_path)
                    root = tree.getroot()
                    joints = root.findall('joint')
                    joint_tree_dict = trip_kinematics.URDFParser._build_joint_tree_dict(joints)

                    for joint in joint_tree_dict.values():
                        transf_kp = chain_kinpy.forward_kinematics(state_to_kinpy(state)) \
                            [joint['child_link']]

                        transf_kp_hom = \
                            trip_kinematics.Utility.hom_translation_matrix(*transf_kp.pos) \
                            .astype('float64')

                        transf_trip_hom = \
                            trip_kinematics.forward_kinematics(robot, joint).astype('float64')

                        assert np.allclose(transf_kp_hom, transf_trip_hom)

                        print(joint['child_link'])

                    print(1)

                except KeyError:
                    print(
                        f'Warning: planar and/or floating joint in {filename}; these are not currently supported. Skipping test case.')

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
