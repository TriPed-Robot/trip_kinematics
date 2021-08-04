from triped import triped_leg, gimbal_joint, extend_motor
from trip_kinematics.Robot import inverse_kinematics
import csv

if __name__ == '__main__':

    filename_input = 'tests/experiments/inverse_2/input_data/matlab_foot_coordinates.csv'
    filename_tip = 'tests/experiments/inverse_2/matlab_output/left_extend_right.csv'

    filename_output = 'tests/experiments/inverse_2/output_data/left_right_extend.csv'

    input_x = []
    input_y = []
    input_z = []

    input_t1_tip = []
    input_t2_tip = []
    input_e_tip = []

    with open(filename_input, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_x.append(float(row[1]))
            input_y.append(float(row[2]))
            input_z.append(float(row[3]))

    with open(filename_tip, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_t1_tip.append(float(row[1]))
            input_t2_tip.append(float(row[3]))
            input_e_tip.append(float(row[2]))

    output_rows = []
    tip = {'t1': 0, 't2': 0, 'ry': 0}
    for i in range(len(input_x)):
        tip['t1'] = input_t1_tip[i]
        tip['t2'] = input_t2_tip[i]
        tip['ry'] = input_e_tip[i]

        gimbal_joint.pass_arguments_g([tip])
        extend_motor.set_actuated_state([{'LL_revolute_joint_ry': tip['ry']}])
        gimbal_joint.set_actuated_state([{'t1': tip['t1'], 't2':tip['t2']}])

        row = inverse_kinematics(
            triped_leg, [input_x[i], input_y[i], input_z[i]])
        output_rows.append([row[0][0][0]['t1'], row[0][0][0]
                            ['t2'], row[1][0][0]['LL_revolute_joint_ry']])

    with open(filename_output, 'w') as f:
        writer = csv.writer(f)
        for row in output_rows:
            writer.writerow(row)
