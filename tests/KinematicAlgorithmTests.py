import unittest
import csv
import os
from trip_kinematics.Robot import inverse_kinematics
import experiments.triped_leg.inverse_kinematics_experiment as tri



robot_type ="triped_leg"

#TODO call experiment script which generates the outputs! 
#     The script is dependent on the type of kinematic algorithms that are tested!

forward_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','endeffector_coordinates.csv')
forward_calculated  = os.path.join('tests','experiments',robot_type,'calculated_solution','endeffector_coordinates.csv')


forward_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv')
forward_calculated  = os.path.join('tests','experiments',robot_type,'calculated_solution','joint_values.csv')

class TestStates(unittest.TestCase):

    def test_trivial_inverse_kinematics(self):
        tri.run(rbt.inverse_kinematics)
        
        return 0


if __name__ == '__main__':
    unittest.main()
