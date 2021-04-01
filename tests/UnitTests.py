import unittest
from trip_kinematics.Joint import State


class TestStates(unittest.TestCase):

    def test_state_constructor_input(self):
        # One variables and one constant
        val_input_1 = {"t1": 1, "t2": 3}
        adjustable_input_1 = ["t1"]

        state_1 = State(values=val_input_1, adjustable=adjustable_input_1)

        self.assertDictEqual(state_1.variables, {"t1": 1}, "")
        self.assertDictEqual(state_1.get_constants(), {"t2": 3}, "")

        # No variables and no constants
        val_input_2 = {}
        adjustable_input_2 = []

        state_2 = State(values=val_input_2, adjustable=adjustable_input_2)

        self.assertDictEqual(state_2.get_constants(), {}, "")
        self.assertDictEqual(state_2.variables, {}, "")


if __name__ == '__main__':
    unittest.main()
