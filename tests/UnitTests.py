import unittest
from trip_kinematics.Joint import JointState


class TestStates(unittest.TestCase):

    def test_joint_state_Constructor(self):

        self.assertRaises(ValueError, JointState, {"b": 0})

        joint_state = JointState({'q0': 1})

        self.assertEqual(joint_state.get_state(), {'q0': 1})


if __name__ == '__main__':
    unittest.main()
