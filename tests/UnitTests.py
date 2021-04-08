import unittest
from trip_kinematics.Joint import JointState
from trip_kinematics.KinematicChainPart import KinematicChainPart


class TestStates(unittest.TestCase):

    def test_JointState_Constructor(self):

        self.assertRaises(ValueError, JointState, {"b": 0})

        joint_state = JointState({'q0': 1})

        self.assertEqual(joint_state.get_state(), {'q0': 1})

    def test_KinematicChain_constructor(self):
        parent = KinematicChainPart("test")
        kinematic_chain = KinematicChainPart("test", parent)

        self.assertEqual(kinematic_chain.get_name(), "test")

        self.assertEqual(kinematic_chain.get_parent(), parent)

        self.assertEqual(parent.get_child(), kinematic_chain)

        self.assertRaises(NotImplementedError, kinematic_chain.get_state)

        self.assertRaises(NotImplementedError, kinematic_chain.set_state, {})

        self.assertRaises(NotImplementedError,
                          kinematic_chain.get_transformation)


if __name__ == '__main__':
    unittest.main()
