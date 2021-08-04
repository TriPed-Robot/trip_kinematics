from triped import triped_leg, gimbal_joint, extend_motor
from trip_kinematics.Robot import forward_kinematics
import csv

if __name__ == '__main__':

    filename_input = 'tests/experiments/forward_1/input_data/left_extend_right.csv'
    filename_output = 'tests/experiments/forward_1/output_data/forward_foot_coordinates.csv'

    input_t1 = []
    input_t2 = []
    input_e = []

    with open(filename_input, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            input_t1.append(float(row[0]))
            input_t2.append(float(row[2]))
            input_e.append(float(row[1]))

    output_rows = []
    tip = {'rx': 0, 'ry': 0, 'rz': 0}
    for i in range(len(input_t1)):
        gimbal_joint.pass_arguments_f([tip])
        gimbal_joint.set_actuated_state([
            {'t1': input_t1[i], 't2': input_t2[i]}])
        # print(gimbal_joint.get_virtual_state())
        extend_motor.set_actuated_state(
            [{'LL_revolute_joint_ry': input_e[i]}])
        row_2 = gimbal_joint.get_transformation() * extend_motor.get_transformation()
        row = forward_kinematics(triped_leg)
        print(row, ' ', row_2.get_translation())

        tip = gimbal_joint.get_virtual_state()[1]
        output_rows.append(row)

    with open(filename_output, 'w') as f:
        writer = csv.writer(f)
        for row in output_rows:
            writer.writerow(row)
