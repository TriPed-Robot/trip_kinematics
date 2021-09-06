import csv
import os
import matplotlib.pyplot as plt

##############################################################################
#  This script contains a number of plotting functions                       #
#  to visualize the resuls of kinematics experiments.                        #
#  They can be used to debug problems with the kinematic algorithms of TriP. #
#  Each function has to be executed from TriPs base Path                     #
##############################################################################

def read_csv(filename, dtype=float):
    with open(filename) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        first = True
        lst = []
        rows = 0
        for row in reader:
            try:
                if first:
                    rows = len(row)
                    for i in range(rows):
                        lst.append([])
                    first = False

                for j in range(rows):
                    lst[j].append(float(row[j]))
            except:
                continue
        return lst



def plot_forward(robot_type):
    forward_reference   = read_csv(os.path.join('tests','experiments',robot_type,'reference_solution','endeffector_coordinates.csv'))
    forward_calculated  = read_csv(os.path.join('tests','experiments',robot_type,'forward_kinematics','endeffector_coordinates.csv'))

    plt.plot(forward_reference[0],label="reference x")
    plt.plot(forward_reference[1],label="reference y")
    plt.plot(forward_reference[2],label="reference z")

    plt.plot(forward_calculated[0],label="calculated x")
    plt.plot(forward_calculated[1],label="calculated y")
    plt.plot(forward_calculated[2],label="calculated z")

    plt.title("Forward kinematics test of robot "+robot_type)
    plt.legend()
    plt.show()


def plot_inverse(robot_type,inv_kin_type):
    inverse_reference   = read_csv(os.path.join('tests','experiments',robot_type,'reference_solution','joint_values.csv'))
    inverse_calculated  = read_csv(os.path.join('tests','experiments',robot_type,"inverse_kinematics",inv_kin_type,'joint_values.csv'))

    for i in range(len(inverse_calculated)):
        plt.plot(inverse_reference[i],label="reference joint "+str(i))
        plt.plot(inverse_calculated[i],label="calculated joint "+str(i))

    plt.title("Inverse kinematics test type "+inv_kin_type+" of robot "+robot_type)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    plot_forward("triped")
    plot_inverse("triped","simple")
