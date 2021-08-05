import unittest
import numpy as np
import csv
import os
from trip_kinematics.Robot import inverse_kinematics
from experiments.inverse_kinematic_experiment import test_robot





def unit_test_forward_kinematics(robot_type,precision):
    forward_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','endeffector_coordinates.csv')
    forward_calculated  = os.path.join('tests','experiments',robot_type,'calculated_solution','endeffector_coordinates.csv')



def unit_test_inverse_kinematics(robot_type,inverse_kinematic_alg,precision):
    test_robot(robot_type,inverse_kinematic_alg)
    inverse_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv')
    inverse_calculated  = os.path.join('tests','experiments',robot_type,inverse_kinematic_alg.__name__,'joint_values.csv')
    
    reference  = []
    calculated = []

    with open(inverse_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            reference.append(np.array([float(row[i]) for i in range(len(row))]))

    with open(inverse_calculated, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            calculated.append(np.array([float(row[i]) for i in range(len(row))]))

    sample_results = [ (np.abs(reference[i]-calculated[i]) < precision).all() for i in range(len(reference))]
    return all(sample_results)

class TestStates(unittest.TestCase):


    def test_trivial_inverse_kinematics(self):
        self.assertTrue(unit_test_inverse_kinematics("triped_leg",inverse_kinematics,0.015))

    def test_forward_kinematics(self):
        #TODO
        return 0
if __name__ == '__main__':
    unittest.main()
