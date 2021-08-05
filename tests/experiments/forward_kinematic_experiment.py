from triped import triped_leg, gimbal_joint, extend_motor
from trip_kinematics.Robot import inverse_kinematics, forward_kinematics
import csv
import os



def test_fwd(robot_name):
    available_robots = ["triped_leg"]
    if robot_name == "triped_leg":
        test_triped_leg()
    else:
        raise KeyError("Robot "+robot_name+"not found in the list of available robots: "+str(available_robots))


def test_triped_leg():

    robot_type ="triped_leg"
    inverse_reference   = os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv')

    forward_calculated  = os.path.join('tests','experiments',robot_type,'forward_kinematics','endeffector_coordinates.csv')



    input_t1 = []
    input_t2 = []
    input_e  = []


    forward_rows = []
    state = {'t1': 0, 't2': 0, 'ry': 0}

    with open(inverse_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_t1.append(float(row[0]))
            input_t2.append(float(row[2]))
            input_e.append(float(row[1]))

    for i in range(len(input_t1)):
        state['t1'] = input_t1[i]
        state['t2'] = input_t2[i]
        state['ry'] = input_e[i]

        extend_motor.set_actuated_state([{'LL_revolute_joint_ry': state['ry']}])
        gimbal_joint.set_actuated_state([{'t1': state['t1'], 't2':state['t2']}])

        row = forward_kinematics(triped_leg)
        forward_rows.append(row)


    with open(forward_calculated, 'w') as f:
        writer = csv.writer(f)
        for row in forward_rows:
            writer.writerow(row)



if __name__=="__main__":
    test_robot("triped_leg",inverse_kinematics)


