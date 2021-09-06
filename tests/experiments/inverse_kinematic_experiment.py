from trip_robots.triped import triped
from trip_kinematics.Robot import SimpleInvKinHandle, inverse_kinematics
import time
import csv
import os



def test_inv(robot_name,inverse_kinematic_type):
    available_robots = ["triped"]
    if robot_name == "triped":
        test_triped(inverse_kinematic_type)
    else:
        raise KeyError("Robot "+robot_name+"not found in the list of available robots: "+str(available_robots))


def test_triped(inverse_kinematic_type):

    robot_type ="triped"
    forward_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','endeffector_coordinates.csv')
    inverse_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv')

    inverse_calculated  = os.path.join('tests','experiments',robot_type,'inverse_kinematics',inverse_kinematic_type,'joint_values.csv')

    inv_kin_handle = SimpleInvKinHandle(triped,'leg0_A_LL_Joint_FCS')

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

    start_time = time.time()

    for i in range(len(input_x)):
        tip['swing_left'] = input_t1_tip[i]
        tip['swing_right'] = input_t2_tip[i]
        tip['ry'] = input_e_tip[i]


        groups = triped.get_groups()
        groups['leg0_closed_chain'].pass_arguments_g([tip])
        triped.set_actuated_state({'leg0_extend_joint_ry': tip['ry'],'leg0_swing_left': tip['swing_left'], 'leg0_swing_right':tip['swing_right']})

        row = inverse_kinematics(triped, 'leg0_A_LL_Joint_FCS',[input_x[i], input_y[i], input_z[i]],inv_kin_handle=inv_kin_handle,type=inverse_kinematic_type)
        inverse_rows.append([row['leg0_swing_left'], row['leg0_extend_joint_ry'],row['leg0_swing_right']])
    
    stop_time = time.time()
    calc_time = stop_time-start_time
    print(robot_type+":\n")
    print(str(len(input_x))+" inverse kinematic calculations of type "+inverse_kinematic_type+" where performed in "+str(calc_time)+" seconds\n")

    with open(inverse_calculated, 'w') as f:
        writer = csv.writer(f)
        for row in inverse_rows:
            writer.writerow(row)






