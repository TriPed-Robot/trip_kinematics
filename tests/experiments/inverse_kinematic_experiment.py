from examples.triped import triped_leg, closed_chain, leg_linear_part
from trip_kinematics.Robot import inverse_kinematics, forward_kinematics
import csv
import os



def test_inv(robot_name,inverse_kinematic_algorithm):
    available_robots = ["triped_leg"]
    if robot_name == "triped_leg":
        test_triped_leg(inverse_kinematic_algorithm)
    else:
        raise KeyError("Robot "+robot_name+"not found in the list of available robots: "+str(available_robots))


def test_triped_leg(inverse_kinematic_algorithm):

    robot_type ="triped_leg"
    forward_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','endeffector_coordinates.csv')
    inverse_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv')

    inverse_calculated  = os.path.join('tests','experiments',robot_type,inverse_kinematic_algorithm.__name__,'joint_values.csv')


    input_x = []
    input_y = []
    input_z = []

    input_t1_tip = []
    input_t2_tip = []
    input_e_tip = []


    inverse_rows = []
    tip = {'swing_left': 0, 'swing_right': 0, 'ry': 0}

    with open(forward_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_x.append(float(row[0]))
            input_y.append(float(row[1]))
            input_z.append(float(row[2]))

    with open(inverse_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_t1_tip.append(float(row[0]))
            input_t2_tip.append(float(row[2]))
            input_e_tip.append(float(row[1]))

    for i in range(len(input_x)):
        tip['swing_left'] = input_t1_tip[i]
        tip['swing_right'] = input_t2_tip[i]
        tip['ry'] = input_e_tip[i]


        closed_chain.pass_arguments_g([tip])
        leg_linear_part.set_actuated_state([{'extend_joint_ry': tip['ry']}])
        closed_chain.set_actuated_state([{'swing_left': tip['swing_left'], 'swing_right':tip['swing_right']}])

        row = inverse_kinematic_algorithm(
            triped_leg, [input_x[i], input_y[i], input_z[i]])
        inverse_rows.append([row[0][0][0]['swing_left'], row[1][0][0]['extend_joint_ry'],row[0][0][0]['swing_right']])


    with open(inverse_calculated, 'w') as f:
        writer = csv.writer(f)
        for row in inverse_rows:
            writer.writerow(row)



if __name__=="__main__":
    test_robot("triped_leg",inverse_kinematics)



