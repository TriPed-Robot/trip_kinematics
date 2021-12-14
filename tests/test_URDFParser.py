import unittest
import os

import kinpy as kp
import numpy as np
from trip_kinematics.URDFParser import align_vectors
import trip_kinematics.URDFParser
from trip_kinematics.Robot import Robot, forward_kinematics


class TestStates(unittest.TestCase):
    def test_simple_inverse_kinematics(self):
        urdf_examples = os.path.join('tests', 'urdf_examples')
        urdf_files = os.listdir(urdf_examples)

        for urdf_file in urdf_files:
            full_path_to_file = os.path.join(urdf_examples, urdf_file)
            chain_URDFParser = trip_kinematics.URDFParser.from_urdf(full_path_to_file)

            try:
                chain_kinpy = kp.build_chain_from_urdf(open(full_path_to_file).read())
                robot = Robot(chain_URDFParser)
                robot.set_actuated_state(state)

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
