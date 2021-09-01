import unittest
import numpy as np
import csv
import os
from trip_kinematics.Robot import inverse_kinematics
from experiments.inverse_kinematic_experiment import test_inv
from experiments.forward_kinematic_experiment import test_fwd





def unit_test_forward_kinematics(robot_type,precision):
    test_fwd(robot_type)
    forward_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','endeffector_coordinates.csv')
    forward_calculated  = os.path.join('tests','experiments',robot_type,'forward_kinematics','endeffector_coordinates.csv')

    reference  = []
    calculated = []

    with open(forward_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            reference.append(np.array([float(row[i]) for i in range(len(row))]))

    with open(forward_calculated, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            calculated.append(np.array([float(row[i]) for i in range(len(row))]))

    sample_results = [ (np.abs(reference[i]-calculated[i]) < precision).all() for i in range(len(reference))]
    return all(sample_results)

def unit_test_inverse_kinematics(robot_type,inverse_kinematic_type,precision):
    test_inv(robot_type,inverse_kinematic_type)
    inverse_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv')
    inverse_calculated  = os.path.join('tests','experiments',robot_type,'inverse_kinematics',inverse_kinematic_type,'joint_values.csv')
    
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


    def test_simple_inverse_kinematics(self):
        self.assertTrue(unit_test_inverse_kinematics("triped","simple",0.015))

    def test_forward_kinematics(self):
        self.assertTrue(unit_test_forward_kinematics("triped",0.1))
if __name__ == '__main__':
    unittest.main()
