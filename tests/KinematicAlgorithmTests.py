import unittest
from trip_kinematics.Robot import Robot, forward_kinematics, inverse_kinematics
from trip_kinematics.KinematicGroup import KinematicGroup, Transformation
from math import radians, degrees
from copy import deepcopy



class TestStates(unittest.TestCase):

    def test_trivial_inverse_kinematics(self):
        return 0


if __name__ == '__main__':
    unittest.main()
