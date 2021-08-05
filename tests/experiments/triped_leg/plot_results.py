#!/usr/bin/env python
# coding: utf-8

# In[1]:


import matplotlib.pyplot as plt
import csv


# In[2]:


def read_csv(filename, dtype=float):
    with open(filename) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        first = True
        lst = []
        rows = 0
        for row in reader:
            if first:
                rows = len(row)
                for i in range(rows):
                    lst.append([])
                first = False

            for j in range(rows):
                lst[j].append(float(row[j]))
        return lst

def get_error(data_1,data_2):
    error = []
    for i in range(len(data_1)):     
        error.append(abs(data_2[i] - data_1[i]))
    return error


# # Inverse Kinematics Test
# This section plots the result of the inverse kinematics test. 
# The output should correspond to the matlab output

# In[3]:


#joint_values = read_csv('calculated_solution/joint_values.csv')
joint_values = read_csv('inverse_kinematics/joint_values.csv')

matlab_joint_values = read_csv('reference_solution/joint_values.csv')
#tips = read_csv('matlab_output/gimbal_extend.csv')


# In[4]:


joint_values_t1 = joint_values[0]
joint_values_t2 = joint_values[1]
joint_values_e  = joint_values[2]

matlab_joint_values_t1 = matlab_joint_values[0]
matlab_joint_values_e  = matlab_joint_values[1]
matlab_joint_values_t2 = matlab_joint_values[2]

error_t1 = get_error(matlab_joint_values_t1,joint_values_t1)
error_t2 = get_error(matlab_joint_values_t2,joint_values_t2)
error_e  = get_error(matlab_joint_values_e,joint_values_e)


# In[5]:


plt.plot(joint_values_t1,label="t1")
plt.plot(joint_values_t2,label="t2")
plt.plot(joint_values_e,label="e")
plt.plot(matlab_joint_values_t1,label="m_t1")
plt.plot(matlab_joint_values_t2,label="m_t2")
plt.plot(matlab_joint_values_e,label="m_e")


plt.title("Inverse Kinematics Trip vs Matlab")
plt.legend(loc='upper right')
plt.show()


# In[6]:


plt.plot(error_t1,label="error_t1")
plt.plot(error_t2,label="error_t2")
plt.plot(error_e,label="error_e")
plt.title("Exp_2 Error")
plt.legend(loc='upper right')
plt.show()


# # Forward Test
# This section plots the result of the forward kinematics test. 
# The output should correspond to the matlab output

# In[7]:


foot_coordinates = read_csv('calculated_solution/endeffector_coordinates.csv')
matlab_foot_coordinates = read_csv('reference_solution/endeffector_coordinates.csv')


# In[8]:


foot_coordinates_x = foot_coordinates[0]
foot_coordinates_y = foot_coordinates[1]
foot_coordinates_z = foot_coordinates[2]

matlab_foot_coordinates_x = matlab_foot_coordinates[0]
matlab_foot_coordinates_y = matlab_foot_coordinates[1]
matlab_foot_coordinates_z = matlab_foot_coordinates[2]

error_x = get_error(matlab_foot_coordinates_x,foot_coordinates_x)
error_y = get_error(matlab_foot_coordinates_y,foot_coordinates_y)
error_z = get_error(matlab_foot_coordinates_z,foot_coordinates_z)


# In[9]:


plt.plot(foot_coordinates_x,label="x")
plt.plot(foot_coordinates_y,label="y")
plt.plot(foot_coordinates_z,label="z")
plt.plot(matlab_foot_coordinates_x,label="m_x")
plt.plot(matlab_foot_coordinates_y,label="m_y")
plt.plot(matlab_foot_coordinates_z,label="m_z")


plt.title("Forward Kinematics Trip vs Matlab")
plt.legend(loc='upper right')
plt.show()

