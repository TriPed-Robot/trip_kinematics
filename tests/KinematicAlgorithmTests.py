import unittest
import numpy as np
import csv
import os
from trip_kinematics.Robot import inverse_kinematics
from experiments.inverse_kinematic_experiment import test_robot



robot_type ="triped_leg"

#TODO call experiment script which generates the outputs! 
#     The script is dependent on the type of kinematic algorithms that are tested!

forward_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','endeffector_coordinates.csv')
forward_calculated  = os.path.join('tests','experiments',robot_type,'calculated_solution','endeffector_coordinates.csv')


inverse_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv')

def unit_test_inverse_kinematics(inverse_kinematic_alg,precision):
    test_robot(robot_type,inverse_kinematic_alg)
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

    print(reference)
    print(calculated)
    sample_results = [ (np.abs(reference[i]-calculated[i]) < precision).all() for i in range(len(reference))]

    return (sample_results == True).all()

class TestStates(unittest.TestCase):


    def test_trivial_inverse_kinematics(self):
        unit_test_inverse_kinematics(inverse_kinematics,1)
        return 0


if __name__ == '__main__':
    unittest.main()
