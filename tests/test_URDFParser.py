import unittest
import os

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


class TestStates(unittest.TestCase):
    def test_simple_inverse_kinematics(self):
        urdf_examples_dir = os.path.join('tests', 'urdf_examples')
        urdf_examples_filenames = os.listdir(urdf_examples_dir)

        for filename in urdf_examples_filenames:
            full_path = os.path.join(urdf_examples_dir, filename)

            with open(full_path, encoding='utf8') as file:
                try:
                    # setup kinpy chain
                    chain_kinpy = kp.build_chain_from_urdf(file.read())
                    state = {
                        joint_name: 0
                        for joint_name in chain_kinpy.get_joint_parameter_names()
                    }
                    chain_kinpy.forward_kinematics(state_to_kinpy(state))

                    # setup TriP robot using the URDF parser
                    chain_trip = trip_kinematics.URDFParser.from_urdf(full_path)
                    robot = trip_kinematics.Robot(chain_trip)
                    robot.set_actuated_state(state_to_trip(state))
                    trip_kinematics.forward_kinematics(robot, 'elbow2_tra')

                    forearm_kp = chain_kinpy.forward_kinematics(state_to_kinpy(state))['forearm']
                    forearm_kp_rot_mat = trip_kinematics.Utility.quat_rotation_matrix(*forearm_kp.rot)
                    forearm_kp_rot_hom = trip_kinematics.Utility.hom_rotation(forearm_kp_rot_mat)
                    forearm_kp_tra_hom = trip_kinematics.Utility.hom_translation_matrix(*forearm_kp.pos)

                    print(1)

                except KeyError:
                    # kinpy does not support planar or floating joints, don't run kinpy test
                    # ? URDFToCasadi instead?
                    pass

        print('hi')

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
