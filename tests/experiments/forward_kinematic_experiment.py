from trip_robots.triped import triped
from trip_kinematics.Robot import forward_kinematics
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
            input_t1.append([float(row[0]),float(row[3]),float(row[6])])
            input_t2.append([float(row[1]),float(row[4]),float(row[7])])
            input_e.append([float(row[2]) ,float(row[5]),float(row[8])])

    start_time = time.time()

    for i in range(len(input_t1)):
        row = []
        for leg_number in [0,1,2]:
            state['swing_left'] = input_t1[i][leg_number]
            state['swing_right'] = input_t2[i][leg_number]
            state['ry'] = input_e[i][leg_number]

            triped.set_actuated_state({'leg'+str(leg_number)+'_extend_joint_ry': state['ry'], 
                                       'leg'+str(leg_number)+'_swing_left': state['swing_left'], 
                                       'leg'+str(leg_number)+'_swing_right':state['swing_right']})

            leg_row = forward_kinematics(triped,'leg'+str(leg_number)+'_A_LL_Joint_FCS')
            row.extend(leg_row[: 3, 3])
        forward_rows.append(row)

    stop_time = time.time()
    calc_time = stop_time-start_time
    print(robot_type+":\n")
    print(str(len(input_t1)*3)+" forward kinematic calculations where performed in "+str(calc_time)+" seconds\n")


    with open(forward_calculated, 'w') as f:
        writer = csv.writer(f)
        for row in forward_rows:
            writer.writerow(row)






