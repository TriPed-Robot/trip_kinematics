from numpy import array
from casadi import cos, sin


def identity_transformation():
    return array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=object)


def hom_translation_matrix(t_x=0, t_y=0, t_z=0):
    return array(
        [[1, 0, 0, t_x], [0, 1, 0, t_y], [0, 0, 1, t_z], [0., 0., 0., 1.]], dtype=object)


def hom_rotation(rotation_matrix):
    matrix = identity_transformation()
    matrix[:3, :3] = rotation_matrix
    return matrix


def quat_rotation_matrix(q_w, q_x, q_y, q_z) -> array:
    """Generates a 3x3 rotation matrix from q quaternion

    Args:
        q_w (float): part of a quaternion [q_w,q_x,q_y,q_z]
        q_x (float): part of a quaternion [q_w,q_x,q_y,q_z]
        q_y (float): part of a quaternion [q_w,q_x,q_y,q_z]
        q_z (float): part of a quaternion [q_w,q_x,q_y,q_z]

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return array([[1-2*(q_y**2+q_z**2), 2*(q_x*q_y-q_z*q_w), 2*(q_x*q_z + q_y*q_w)], [2*(q_x*q_y + q_z*q_w), 1-2*(q_x**2+q_z**2), 2*(q_y*q_z - q_x*q_w)], [2*(q_x*q_z-q_y*q_w), 2*(q_y*q_z+q_x*q_w), 1-2*(q_x**2+q_y**2)]], dtype=object)


def x_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the x axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return array([[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]], dtype=object)


def y_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the y axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return array([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]], dtype=object)


def z_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the z axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]], dtype=object)


def get_rotation(matrix):
    """Returns the 3x3 rotation matrix of the :py:class`TransformationMatrix`

    Returns:
        numpy.array: The 3x3 rotation matrix
    """
    return matrix[: 3, : 3]


def get_translation(matrix):
    """Returns the translation of the :py:class`TransformationMatrix`

    Returns:
        numpy array: The 3 dimensional translation
    """
    return matrix[: 3, 3]
