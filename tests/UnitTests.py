import unittest
from trip_kinematics.KinematicChainPart import KinematicChainPart
from trip_kinematics.KinematicObject import KinematicObject


class TestStates(unittest.TestCase):

    def test_KinematicChainPart(self):
        parent = KinematicChainPart("test")
        kinematic_chain = KinematicChainPart("test", parent)

        self.assertEqual(kinematic_chain.get_name(), "test")

        self.assertEqual(kinematic_chain.get_parent(), parent)

        self.assertEqual(parent.get_child(), kinematic_chain)

        self.assertRaises(NotImplementedError, kinematic_chain.get_state)

        self.assertRaises(NotImplementedError, kinematic_chain.set_state, {})

        self.assertRaises(NotImplementedError,
                          kinematic_chain.get_transformation)

    def test_KinematicObject(self):
        #self.assertRaises(ValueError, KinematicObject, {})
        pass


if __name__ == '__main__':
    unittest.main()
