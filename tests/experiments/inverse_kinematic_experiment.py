from trip_robots.triped import triped
import time
import csv
import os


def inv_test(robot_name, inverse_kinematic_solver):
    available_robots = ["triped"]
    if robot_name == "triped":
        test_triped(inverse_kinematic_solver)
    else:
        raise KeyError(
            "Robot "+robot_name+"not found in the list of available robots: "+str(available_robots))


def test_triped(inverse_kinematic_solver):

    robot_type = "triped"
    forward_reference = os.path.join(
        'tests', 'experiments', robot_type, 'reference_solution', 'endeffector_coordinates.csv')
    inverse_reference = os.path.join(
        'tests', 'experiments', robot_type, 'reference_solution', 'joint_values.csv')

    inverse_calculated = os.path.join('tests', 'experiments', robot_type,
                                      'inverse_kinematics', inverse_kinematic_solver.__name__,
                                      'joint_values.csv')

    inv_kin_solver = [inverse_kinematic_solver(triped, 'leg_0_A_LL_Joint_FCS', update_robot=False),
                      inverse_kinematic_solver(
                          triped, 'leg_1_A_LL_Joint_FCS', update_robot=False),
                      inverse_kinematic_solver(triped, 'leg_2_A_LL_Joint_FCS', update_robot=False)]

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
            input_x.append([float(row[0]), float(row[3]), float(row[6])])
            input_y.append([float(row[1]), float(row[4]), float(row[7])])
            input_z.append([float(row[2]), float(row[5]), float(row[8])])

    with open(inverse_reference, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_t1_tip.append([float(row[0]), float(row[3]), float(row[6])])
            input_e_tip.append([float(row[1]), float(row[4]), float(row[7])])
            input_t2_tip.append([float(row[2]), float(row[5]), float(row[8])])

    start_time = time.time()

    for i in range(len(input_x)):
        row = []
        for leg_number in [0, 1, 2]:
            tip['leg_'+str(leg_number) +
                '_swing_left'] = input_t1_tip[i][leg_number]
            tip['leg_'+str(leg_number) +
                '_swing_right'] = input_t2_tip[i][leg_number]

            leg_row = inv_kin_solver[leg_number].solve_actuated(
                target=[input_x[i][leg_number],
                        input_y[i][leg_number],
                        input_z[i][leg_number]],
                mapping_argument={'leg_'+str(leg_number)+'_closed_chain': [tip]})

            row.extend([leg_row['leg_'+str(leg_number)+'_swing_left'],
                       leg_row['leg_'+str(leg_number)+'_extend_joint_ry'],
                       leg_row['leg_'+str(leg_number)+'_swing_right']])
        inverse_rows.append(row)

    stop_time = time.time()
    calc_time = stop_time-start_time
    print(robot_type+":\n")
    print(str(len(input_x)*3)+" inverse kinematic calculations of type " +
          inverse_kinematic_solver.__name__+" where performed in "+str(calc_time)+" seconds\n")

    with open(inverse_calculated, 'w', newline='') as f:
        writer = csv.writer(f)
        for row in inverse_rows:
            writer.writerow(row)
