from examples.triped import triped
from trip_kinematics.Robot import inverse_kinematics, forward_kinematics
import time
import csv
import os



def test_fwd(robot_name):
    available_robots = ["triped"]
    if robot_name == "triped":
        test_triped_leg()
    else:
        raise KeyError("Robot "+robot_name+"not found in the list of available robots: "+str(available_robots))


def test_triped_leg():

    robot_type ="triped"
    inverse_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv')

    forward_calculated  = os.path.join('tests','experiments',robot_type,'forward_kinematics','endeffector_coordinates.csv')



    input_t1 = []
    input_t2 = []
    input_e  = []


    forward_rows = []
    state = {'swing_left': 0, 'swing_right': 0, 'ry': 0}

    with open(inverse_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_t1.append(float(row[0]))
            input_t2.append(float(row[2]))
            input_e.append(float(row[1]))

    start_time = time.time()

    for i in range(len(input_t1)):
        state['swing_left'] = input_t1[i]
        state['swing_right'] = input_t2[i]
        state['ry'] = input_e[i]

        triped.set_actuated_state({'leg0_extend_joint_ry': state['ry'], 'leg0_swing_left': state['swing_left'], 'leg0_swing_right':state['swing_right']})

        row = forward_kinematics(triped,'leg0_A_LL_Joint_FCS')
        forward_rows.append(row[: 3, 3])

    stop_time = time.time()
    calc_time = stop_time-start_time
    print(robot_type+":\n")
    print(str(len(input_t1))+" forward kinematic calculations where performed in "+str(calc_time)+" seconds\n")


    with open(forward_calculated, 'w') as f:
        writer = csv.writer(f)
        for row in forward_rows:
            writer.writerow(row)






