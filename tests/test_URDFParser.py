import unittest
import os

import kinpy as kp
from trip_kinematics.URDFParser import from_urdf
from trip_kinematics.Robot import Robot, forward_kinematics


class TestStates(unittest.TestCase):
    def test_simple_inverse_kinematics(self):
        urdf_examples = os.path.join('tests', 'urdf_examples')
        urdf_files = os.listdir(urdf_examples)

        for urdf_file in urdf_files:
            full_path_to_file = os.path.join(urdf_examples, urdf_file)
            chain_URDFParser = from_urdf(full_path_to_file)

            try:
                chain_kinpy = kp.build_chain_from_urdf(open(full_path_to_file).read())
                robot = Robot(chain_URDFParser)
                robot.set_actuated_state(state)

            except KeyError:
                # kinpy does not support planar or floating joints, don't run kinpy test
                # ? URDFToCasadi instead?
                pass

        print('hi')


if __name__ == '__main__':
    unittest.main()
